# PlatformIO pre-script: fixes in the Zephyr framework package, applied before every build (idempotent).
#
# can_mcan.c: the TX event handler calls a TX buffer's callback without checking it. When the controller went bus-off,
# the bus-off handler has already completed that buffer (callback NULL), and a TX event for it arriving afterwards jumps
# to address 0: a CPU fault in the CAN interrupt (seen here every few minutes while holding). Upstream Zephyr fixed it
# after 4.4 with this check ("The bus-off handler may already have completed this Tx Buffer").

import os

Import("env")  # noqa: F821 (SCons)

OLD = """\t\ttx_cb = cbs->tx[tx_idx].function;
\t\tuser_data = cbs->tx[tx_idx].user_data;
\t\tcbs->tx[tx_idx].function = NULL;

\t\tk_sem_give(&data->tx_sem);

\t\ttx_cb(dev, 0, user_data);
"""

NEW = """\t\ttx_cb = cbs->tx[tx_idx].function;

\t\t/* The bus-off handler may already have completed this Tx Buffer (can-do-gyro patch) */
\t\tif (tx_cb != NULL) {
\t\t\tuser_data = cbs->tx[tx_idx].user_data;
\t\t\tcbs->tx[tx_idx].function = NULL;

\t\t\tk_sem_give(&data->tx_sem);

\t\t\ttx_cb(dev, 0, user_data);
\t\t}
"""

# The Zephyr the build uses: Seeed's platform takes the board's build.zephyr.package (framework-zephyr-nrf54lm20 for
# the XIAO STM32C5), not framework-zephyr
package = env.BoardConfig().get("build.zephyr.package", "") or "framework-zephyr"  # noqa: F821
path = os.path.join(env.PioPlatform().get_package_dir(package), "drivers", "can", "can_mcan.c")  # noqa: F821
with open(path, "rb") as f:
    raw = f.read().decode()
crlf = "\r\n" in raw
text = raw.replace("\r\n", "\n")
if "can-do-gyro patch" not in text:
    if text.count(OLD) != 1:
        raise SystemExit("patch_zephyr.py: can_mcan.c doesn't match, check the TX event handler by hand: " + path)
    text = text.replace(OLD, NEW)
    if not os.path.exists(path + ".orig"):
        with open(path + ".orig", "wb") as f:
            f.write(raw.encode())
    with open(path, "wb") as f:
        f.write((text.replace("\n", "\r\n") if crlf else text).encode())
    print("patch_zephyr.py: patched " + path)
