//! Static IP configuration for the instrument LAN: NetworkManager on Linux,
//! PowerShell on Windows.
//!
//! This changes host-wide network state, so it deliberately refuses to touch the
//! interface that currently carries the default route: a typo there would take
//! the machine off the network with no way back from inside the app.

use std::net::Ipv4Addr;

/// One wired NetworkManager profile and how its IPv4 is configured right now.
#[derive(Debug, Clone, PartialEq)]
pub struct LanProfile {
    pub device: String,
    pub profile: String,
    pub method: String,
    pub addresses: String,
    pub carries_default_route: bool,
}

impl LanProfile {
    pub fn label(&self) -> String {
        format!("{} \u{2014} {}", self.device, self.profile)
    }
}

/// Lists wired profiles that can be reconfigured.
pub fn list_wired() -> Result<Vec<LanProfile>, String> {
    platform::list_wired()
}

/// Switches a profile to a fixed address with no gateway, so the instrument LAN
/// never competes with the uplink for the default route.
pub fn apply_static(target: &LanProfile, cidr: &str) -> Result<String, String> {
    let (address, prefix) = validate_cidr(cidr)?;
    guard_default_route(target)?;
    platform::set_static(target, address, prefix)?;
    Ok(format!(
        "{} set to static {address}/{prefix}",
        target.device
    ))
}

/// Puts a profile back on DHCP, for when the static address was a mistake.
pub fn apply_dhcp(target: &LanProfile) -> Result<String, String> {
    guard_default_route(target)?;
    platform::set_dhcp(target)?;
    Ok(format!("{} set back to DHCP", target.device))
}

fn guard_default_route(target: &LanProfile) -> Result<(), String> {
    if target.carries_default_route {
        return Err(format!(
            "{} carries the default route; reconfiguring it from here would cut this machine off the network",
            target.device
        ));
    }
    Ok(())
}

/// Accepts `a.b.c.d/prefix` only. A bare address is rejected rather than guessed
/// at, because the wrong prefix silently splits the instrument off the subnet.
pub fn validate_cidr(value: &str) -> Result<(Ipv4Addr, u8), String> {
    let (address, prefix) = value
        .trim()
        .split_once('/')
        .ok_or("address must include a prefix, for example 192.168.1.50/24")?;
    let address: Ipv4Addr = address
        .trim()
        .parse()
        .map_err(|_| format!("`{address}` is not an IPv4 address"))?;
    let prefix: u8 = prefix
        .trim()
        .parse()
        .map_err(|_| format!("`{prefix}` is not a prefix length"))?;
    if !(1..=32).contains(&prefix) {
        return Err("prefix length must be between 1 and 32".to_owned());
    }
    if address.is_loopback() || address.is_multicast() || address.is_unspecified() {
        return Err(format!("{address} cannot be assigned to an interface"));
    }
    Ok((address, prefix))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cidr_needs_a_prefix_and_a_usable_address() {
        assert_eq!(
            validate_cidr(" 192.168.1.50/24 ").unwrap(),
            (Ipv4Addr::new(192, 168, 1, 50), 24)
        );
        assert!(validate_cidr("192.168.1.50").is_err());
        assert!(validate_cidr("192.168.1.999/24").is_err());
        assert!(validate_cidr("192.168.1.50/33").is_err());
        assert!(validate_cidr("192.168.1.50/0").is_err());
        assert!(validate_cidr("127.0.0.1/8").is_err());
        assert!(validate_cidr("0.0.0.0/24").is_err());
    }

    #[test]
    fn the_default_route_interface_is_never_reconfigured() {
        let uplink = LanProfile {
            device: "wlan0".to_owned(),
            profile: "office".to_owned(),
            method: "auto".to_owned(),
            addresses: String::new(),
            carries_default_route: true,
        };
        assert!(apply_static(&uplink, "192.168.1.50/24").is_err());
        assert!(apply_dhcp(&uplink).is_err());
    }
}

#[cfg(target_os = "linux")]
mod platform {
    use super::LanProfile;
    use std::fs;
    use std::net::Ipv4Addr;
    use std::process::Command;

    pub fn list_wired() -> Result<Vec<LanProfile>, String> {
        let devices = run(&["-t", "-f", "DEVICE,TYPE,CONNECTION", "device", "status"])?;
        let default_device = default_route_device();
        let mut profiles = Vec::new();
        for row in parse_terse(&devices) {
            let [device, kind, profile] = <[String; 3]>::try_from(row).unwrap_or_default();
            if kind != "ethernet" || profile.is_empty() || profile == "--" {
                continue;
            }
            let (method, addresses) = ipv4_of(&profile).unwrap_or_default();
            profiles.push(LanProfile {
                carries_default_route: default_device.as_deref() == Some(device.as_str()),
                device,
                profile,
                method,
                addresses,
            });
        }
        Ok(profiles)
    }

    pub fn set_static(target: &LanProfile, address: Ipv4Addr, prefix: u8) -> Result<(), String> {
        run(&[
            "connection",
            "modify",
            &target.profile,
            "ipv4.method",
            "manual",
            "ipv4.addresses",
            &format!("{address}/{prefix}"),
            "ipv4.gateway",
            "",
            "ipv4.never-default",
            "yes",
        ])?;
        run(&["--wait", "20", "connection", "up", &target.profile])?;
        Ok(())
    }

    pub fn set_dhcp(target: &LanProfile) -> Result<(), String> {
        run(&[
            "connection",
            "modify",
            &target.profile,
            "ipv4.method",
            "auto",
            "ipv4.addresses",
            "",
            "ipv4.gateway",
            "",
            "ipv4.never-default",
            "no",
        ])?;
        run(&["--wait", "20", "connection", "up", &target.profile])?;
        Ok(())
    }

    fn ipv4_of(profile: &str) -> Option<(String, String)> {
        let output = run(&[
            "-t",
            "-f",
            "ipv4.method,ipv4.addresses",
            "connection",
            "show",
            profile,
        ])
        .ok()?;
        let mut method = String::new();
        let mut addresses = String::new();
        for line in output.lines() {
            let Some((key, value)) = line.split_once(':') else {
                continue;
            };
            match key {
                "ipv4.method" => method = value.trim().to_owned(),
                "ipv4.addresses" => addresses = value.trim().to_owned(),
                _ => {}
            }
        }
        Some((method, addresses))
    }

    fn run(args: &[&str]) -> Result<String, String> {
        let output = Command::new("nmcli")
            .args(args)
            .output()
            .map_err(|error| format!("cannot run nmcli: {error}"))?;
        if output.status.success() {
            Ok(String::from_utf8_lossy(&output.stdout).into_owned())
        } else {
            let message = String::from_utf8_lossy(&output.stderr);
            let message = message.trim();
            Err(if message.is_empty() {
                format!("nmcli failed with {}", output.status)
            } else {
                message.to_owned()
            })
        }
    }

    /// Interface carrying the IPv4 default route, read straight from the kernel
    /// so the guard does not depend on nmcli agreeing.
    fn default_route_device() -> Option<String> {
        let table = fs::read_to_string("/proc/net/route").ok()?;
        table.lines().skip(1).find_map(|line| {
            let mut fields = line.split_whitespace();
            let interface = fields.next()?;
            let destination = fields.next()?;
            (destination == "00000000").then(|| interface.to_owned())
        })
    }

    /// Splits one `nmcli -t` record, honouring its `\:` and `\\` escapes.
    fn parse_terse(output: &str) -> Vec<Vec<String>> {
        output
            .lines()
            .filter(|line| !line.trim().is_empty())
            .map(|line| {
                let mut fields = vec![String::new()];
                let mut escaped = false;
                for character in line.chars() {
                    match character {
                        _ if escaped => {
                            escaped = false;
                            fields.last_mut().expect("always one field").push(character);
                        }
                        '\\' => escaped = true,
                        ':' => fields.push(String::new()),
                        _ => fields.last_mut().expect("always one field").push(character),
                    }
                }
                fields
            })
            .collect()
    }

    #[cfg(test)]
    mod tests {
        use super::parse_terse;

        #[test]
        fn terse_rows_keep_escaped_colons_inside_a_field() {
            assert_eq!(
                parse_terse("enp0s31f6:ethernet:Wired connection 1\n"),
                vec![vec![
                    "enp0s31f6".to_owned(),
                    "ethernet".to_owned(),
                    "Wired connection 1".to_owned()
                ]]
            );
            assert_eq!(
                parse_terse("wlan0:wifi:LUNAR\\:iot2"),
                vec![vec![
                    "wlan0".to_owned(),
                    "wifi".to_owned(),
                    "LUNAR:iot2".to_owned()
                ]]
            );
        }
    }
}

#[cfg(target_os = "windows")]
mod platform {
    //! PowerShell rather than `netsh`: cmdlet output keys are stable, while
    //! netsh prints localised field names that break parsing on a non-English
    //! Windows. Every call here needs an elevated process.
    use super::LanProfile;
    use std::net::Ipv4Addr;
    use std::process::Command;

    pub fn list_wired() -> Result<Vec<LanProfile>, String> {
        let script = "\
Get-NetAdapter -Physical | Where-Object { $_.Status -eq 'Up' } | ForEach-Object {
  $a = Get-NetIPAddress -InterfaceIndex $_.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -First 1
  $i = Get-NetIPInterface -InterfaceIndex $_.ifIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue | Select-Object -First 1
  $addr = if ($a) { \"$($a.IPAddress)/$($a.PrefixLength)\" } else { '' }
  $method = if ($i -and $i.Dhcp -eq 'Enabled') { 'auto' } else { 'manual' }
  \"$($_.Name)`t$method`t$addr\"
}";
        let default_device = default_route_device();
        let mut profiles = Vec::new();
        for line in run(script)?.lines() {
            let mut fields = line.split('\t');
            let (Some(device), Some(method), Some(addresses)) =
                (fields.next(), fields.next(), fields.next())
            else {
                continue;
            };
            let device = device.trim().to_owned();
            if device.is_empty() {
                continue;
            }
            profiles.push(LanProfile {
                carries_default_route: default_device.as_deref() == Some(device.as_str()),
                profile: device.clone(),
                device,
                method: method.trim().to_owned(),
                addresses: addresses.trim().to_owned(),
            });
        }
        Ok(profiles)
    }

    pub fn set_static(target: &LanProfile, address: Ipv4Addr, prefix: u8) -> Result<(), String> {
        let alias = quote(&target.device);
        // No -DefaultGateway: the instrument LAN must never take the default
        // route away from the uplink.
        run(&format!(
            "Remove-NetIPAddress -InterfaceAlias {alias} -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue; \
             Remove-NetRoute -InterfaceAlias {alias} -DestinationPrefix '0.0.0.0/0' -Confirm:$false -ErrorAction SilentlyContinue; \
             Set-NetIPInterface -InterfaceAlias {alias} -AddressFamily IPv4 -Dhcp Disabled; \
             New-NetIPAddress -InterfaceAlias {alias} -AddressFamily IPv4 -IPAddress {address} -PrefixLength {prefix}"
        ))?;
        Ok(())
    }

    pub fn set_dhcp(target: &LanProfile) -> Result<(), String> {
        let alias = quote(&target.device);
        run(&format!(
            "Remove-NetIPAddress -InterfaceAlias {alias} -AddressFamily IPv4 -Confirm:$false -ErrorAction SilentlyContinue; \
             Set-NetIPInterface -InterfaceAlias {alias} -AddressFamily IPv4 -Dhcp Enabled; \
             Set-DnsClientServerAddress -InterfaceAlias {alias} -ResetServerAddresses"
        ))?;
        Ok(())
    }

    fn default_route_device() -> Option<String> {
        let alias = run(
            "(Get-NetRoute -DestinationPrefix '0.0.0.0/0' -ErrorAction SilentlyContinue | \
             Sort-Object RouteMetric | Select-Object -First 1).InterfaceAlias",
        )
        .ok()?;
        let alias = alias.trim().to_owned();
        (!alias.is_empty()).then_some(alias)
    }

    /// Single-quoted PowerShell literal; an embedded quote is doubled.
    fn quote(value: &str) -> String {
        format!("'{}'", value.replace('\'', "''"))
    }

    fn run(script: &str) -> Result<String, String> {
        let output = Command::new("powershell")
            .args(["-NoProfile", "-NonInteractive", "-Command", script])
            .output()
            .map_err(|error| format!("cannot run powershell: {error}"))?;
        if output.status.success() {
            Ok(String::from_utf8_lossy(&output.stdout).into_owned())
        } else {
            let message = String::from_utf8_lossy(&output.stderr);
            let message = message.trim();
            Err(if message.is_empty() {
                format!(
                    "powershell failed with {} (run the app as Administrator)",
                    output.status
                )
            } else {
                message.to_owned()
            })
        }
    }
}

#[cfg(not(any(target_os = "linux", target_os = "windows")))]
mod platform {
    use super::LanProfile;
    use std::net::Ipv4Addr;

    const UNSUPPORTED: &str = "LAN configuration is only implemented for Linux and Windows";

    pub fn list_wired() -> Result<Vec<LanProfile>, String> {
        Ok(Vec::new())
    }

    pub fn set_static(_: &LanProfile, _: Ipv4Addr, _: u8) -> Result<(), String> {
        Err(UNSUPPORTED.to_owned())
    }

    pub fn set_dhcp(_: &LanProfile) -> Result<(), String> {
        Err(UNSUPPORTED.to_owned())
    }
}
