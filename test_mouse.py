#!/usr/bin/env python3
"""
Mouse tracking test for Diamond OS virtio-tablet driver.

Boots the OS headlessly, waits for the shell, starts the piano app,
then injects absolute X mouse events via QMP and reads the serial log
to verify that increasing ABS_X values produce strictly increasing
screen-X coordinates.
"""

import socket, json, time, sys, re, os, subprocess, signal, tempfile

QMP_SOCK = '/tmp/diamond_qmp.sock'
SERIAL_LOG = '/tmp/diamond_serial.log'

# ── QMP helpers ──────────────────────────────────────────────────────────────

def qmp_recv(s):
    buf = b''
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
        try:
            json.loads(buf.decode())
            return buf.decode()
        except json.JSONDecodeError:
            pass
    return buf.decode()

def qmp_cmd(s, cmd):
    msg = json.dumps(cmd) + '\n'
    s.sendall(msg.encode())
    time.sleep(0.05)
    # drain any response
    s.setblocking(False)
    resp = b''
    try:
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            resp += chunk
    except BlockingIOError:
        pass
    s.setblocking(True)
    return resp.decode()

def qmp_connect(path, timeout=30):
    s = socket.socket(socket.AF_UNIX)
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            s.connect(path)
            s.setblocking(True)
            s.settimeout(2.0)
            # read greeting
            try: s.recv(4096)
            except: pass
            # negotiate
            qmp_cmd(s, {"execute": "qmp_capabilities"})
            return s
        except (FileNotFoundError, ConnectionRefusedError):
            time.sleep(0.5)
    raise RuntimeError(f"Could not connect to QMP socket at {path}")

def send_key(s, qcode):
    qmp_cmd(s, {"execute": "input-send-event", "arguments": {
        "events": [{"type": "key", "data": {"down": True,
                                             "key": {"type": "qcode",
                                                     "data": qcode}}}]}})
    time.sleep(0.03)
    qmp_cmd(s, {"execute": "input-send-event", "arguments": {
        "events": [{"type": "key", "data": {"down": False,
                                             "key": {"type": "qcode",
                                                     "data": qcode}}}]}})
    time.sleep(0.03)

def type_string(s, text):
    QCODE = {
        'a':'a','b':'b','c':'c','d':'d','e':'e','f':'f','g':'g','h':'h',
        'i':'i','j':'j','k':'k','l':'l','m':'m','n':'n','o':'o','p':'p',
        'q':'q','r':'r','s':'s','t':'t','u':'u','v':'v','w':'w','x':'x',
        'y':'y','z':'z','0':'0','1':'1','2':'2','3':'3','4':'4','5':'5',
        '6':'6','7':'7','8':'8','9':'9','\n':'ret',' ':'spc',
    }
    for c in text:
        code = QCODE.get(c)
        if code:
            send_key(s, code)

def send_abs(s, x, y=28000):
    """Send an absolute mouse event (x in 0..32767, y defaults to piano area)."""
    qmp_cmd(s, {"execute": "input-send-event", "arguments": {
        "events": [
            {"type": "abs", "data": {"axis": "x", "value": x}},
            {"type": "abs", "data": {"axis": "y", "value": y}},
        ]
    }})

# ── Main test ────────────────────────────────────────────────────────────────

def main():
    # Clean up previous run
    for p in (QMP_SOCK, SERIAL_LOG):
        try: os.unlink(p)
        except FileNotFoundError: pass

    KVM = ['-enable-kvm', '-cpu', 'host'] if os.path.exists('/dev/kvm') and os.access('/dev/kvm', os.W_OK) else ['-cpu', 'qemu64']

    qemu_cmd = [
        'qemu-system-x86_64',
        '-machine', 'q35',
        '-bios', '/usr/share/ovmf/OVMF.fd',
        '-cdrom', 'diamond.iso',
        '-m', '512M',
        '-smp', '4',
        '-device', 'bochs-display,xres=2560,yres=1440,vgamem=16777216',
        # VNC keeps the display + input subsystem alive without needing a screen
        '-display', 'vnc=127.0.0.1:5',
        '-serial', f'file:{SERIAL_LOG}',
        '-netdev', 'user,id=net0',
        '-device', 'e1000,netdev=net0',
        '-device', 'intel-hda',
        '-device', 'hda-duplex',
        '-device', 'virtio-tablet-pci',
        '-device', 'virtio-keyboard-pci',
        '-qmp', f'unix:{QMP_SOCK},server,nowait',
    ] + KVM

    print(f"[*] Starting QEMU: {' '.join(qemu_cmd)}")
    proc = subprocess.Popen(qemu_cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)

    try:
        print("[*] Waiting for QMP socket…")
        qmp = qmp_connect(QMP_SOCK, timeout=40)
        print("[*] QMP connected. Waiting 12 s for OS to boot and shell to appear…")
        time.sleep(12)

        print("[*] Typing 'mtest' + Enter…")
        type_string(qmp, 'mtest\n')
        time.sleep(0.5)   # let mtest start its poll loop

        # Inject a sweep of 10 equally-spaced ABS_X values, left to right.
        # Expected screen-x = raw * 2560 / 32768
        test_raws = [0, 3276, 6553, 9830, 13107, 16384, 19660, 22937, 26214, 32767]
        expected  = [r * 2560 // 32768 for r in test_raws]

        print(f"[*] Injecting {len(test_raws)} rightward ABS_X events via QMP…")
        for raw in test_raws:
            send_abs(qmp, raw)
            time.sleep(0.5)

        # Inject 10 more events to confirm no leftward regression
        print(f"[*] Injecting 10 more events to ensure monotonicity…")
        for raw in reversed(test_raws[:-1]):   # these go left — cursor follows
            send_abs(qmp, raw)
            time.sleep(0.3)

        time.sleep(2.0)   # let mtest finish draining
        print("[*] Done injecting. Reading serial log…")

    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill()

    # ── Parse serial log ─────────────────────────────────────────────────────
    try:
        log = open(SERIAL_LOG, 'r', errors='replace').read()
    except FileNotFoundError:
        print("FAIL: serial log not created — QEMU may have crashed on boot.")
        sys.exit(1)

    # Debug lines come from drain_tablet; mtest lines come from the shell command.
    abs_lines = [l for l in log.splitlines() if l.startswith('ABS_X raw=')]
    mtest_lines = [l for l in log.splitlines() if re.match(r'^x=\d+ y=\d+', l)]

    print(f"\n[*] Serial log summary:")
    print(f"    drain_tablet debug lines : {len(abs_lines)}")
    print(f"    mtest output lines       : {len(mtest_lines)}")

    if not abs_lines and not mtest_lines:
        print("\nFAIL: No ABS_X events seen in serial output.")
        print("      Either the virtio-tablet was not detected, or QMP abs events")
        print("      are not routed to it.  Last 40 serial lines:")
        for l in log.splitlines()[-40:]:
            print("  ", repr(l))
        sys.exit(1)

    # Prefer drain_tablet debug lines (have raw value); fall back to mtest.
    parsed = []
    if abs_lines:
        print(f"\n[*] drain_tablet ABS_X events:")
        for line in abs_lines:
            m = re.match(r'ABS_X raw=(\d+) x=(\d+)', line)
            if m:
                raw, x = int(m.group(1)), int(m.group(2))
                exp = raw * 2560 // 32768
                ok = (x == exp)
                parsed.append((raw, x, exp, ok))
                tag = "OK" if ok else "WRONG"
                print(f"  raw={raw:5d}  got x={x:4d}  expected={exp:4d}  [{tag}]")
    else:
        print(f"\n[*] mtest poll output (no raw values available):")
        prev_x = -1
        for line in mtest_lines:
            m = re.match(r'x=(\d+)', line)
            if m:
                x = int(m.group(1))
                ok = (x >= prev_x)
                parsed.append((0, x, 0, ok))
                tag = "OK" if ok else "WENT LEFT"
                print(f"  x={x:4d}  [{tag}]")
                prev_x = x

    if not parsed:
        print("FAIL: Could not parse any ABS_X lines.")
        sys.exit(1)

    # Check monotonicity of the x values we injected (filter to our injected raws)
    our_points = [(raw, x, exp, ok) for (raw, x, exp, ok) in parsed
                  if raw in test_raws]

    wrong = [p for p in parsed if not p[3]]
    print()
    if wrong:
        print(f"FAIL: {len(wrong)} event(s) had wrong x mapping.")
        sys.exit(1)

    xs = [x for (_, x, _, _) in our_points]
    non_increasing = [(xs[i], xs[i+1]) for i in range(len(xs)-1) if xs[i+1] < xs[i]]
    if non_increasing:
        print(f"FAIL: Non-monotone x sequence (went left): {non_increasing}")
        sys.exit(1)

    print(f"PASS: all {len(parsed)} ABS_X events mapped correctly and x is "
          f"monotonically non-decreasing for the injected sweep.")

if __name__ == '__main__':
    main()
