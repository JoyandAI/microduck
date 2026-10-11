# Disable sleep

On a Radxa Zero 3W running the desktop image, an idle board suspends itself. Radxa support
identified that state by the supply: about **7.5 V at 0.2 A**. That is suspend, not power-off —
the board is still on the rail, drawing the sleep current, and it comes back without a clean
shutdown. The journal ends on a NetworkManager disconnect, the boot partition is left dirty
(`Volume was not properly unmounted`), and there is no `reboot`, power-key, or panic record.

A dark panel is a different thing. DPMS only switches the display off; the board keeps running.
The 0.2 A figure is suspend-to-RAM.

KDE PowerDevil is what asks for it. In `~/.config/powermanagementprofilesrc` the
`SuspendSession` idle times are 15 minutes on AC (`900000`), 10 minutes on battery (`600000`),
and 5 minutes on low battery (`300000`). `suspendType=1` is suspend-to-RAM. systemd will still
honour that request until its sleep targets are masked.

## Stop systemd from suspending

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

```bash
sudo mkdir -p /etc/systemd/logind.conf.d
```

```bash
sudo tee /etc/systemd/logind.conf.d/no-suspend.conf >/dev/null << 'EOF'
[Login]
HandleSuspendKey=ignore
HandleSuspendKeyLongPress=ignore
HandleHibernateKey=ignore
HandleHibernateKeyLongPress=ignore
HandleLidSwitch=ignore
HandleLidSwitchExternalPower=ignore
IdleAction=ignore
EOF
```

The mask takes effect immediately. Leave `systemd-logind` running: restarting it drops the
desktop session. The drop-in is read the next time logind starts, which is the next boot. Until
then the masked targets are what refuses the suspend.

Check:

```bash
systemctl is-enabled sleep.target suspend.target hibernate.target hybrid-sleep.target
```

```bash
busctl call org.freedesktop.login1 /org/freedesktop/login1 org.freedesktop.login1.Manager CanSuspend
```

The four targets report `masked`. `CanSuspend` answers `s "no"`.

A board without KDE is done at this point. `IdleAction` already defaults to `ignore`; the mask
is what keeps a later desktop, or a manual `systemctl suspend`, from putting the board to sleep.

## Stop PowerDevil from asking

On the desktop image, edit the logged-in user's power profile. For the `radxa` user that file is
`/home/radxa/.config/powermanagementprofilesrc`. Each `SuspendSession` section gets
`idleTime=0`, which is PowerDevil's "do not suspend". The screen-blank timers
(`DPMSControl`) stay as they are.

```bash
sed -i '/\[SuspendSession\]/,/^\[/ s/^idleTime=.*/idleTime=0/' ~/.config/powermanagementprofilesrc
```

PowerDevil has already read the old timers. Restart it in the graphical session:

```bash
sudo -u radxa env XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus systemctl --user restart plasma-powerdevil.service
```

`1000` is `radxa`'s uid. Another user needs that uid and the matching `/run/user/<uid>`.

Confirm the three suspend timers, and only those, are zero. The DPMS and dim lines stay non-zero:

```bash
grep -n idleTime ~/.config/powermanagementprofilesrc
```
