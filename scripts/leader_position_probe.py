"""Read-only: print the leader's live position every 0.3s for 10s.
No follower, no writes -- isolates whether the leader is actually
streaming fresh readings or stuck repeating one stale value (as seen
in teleop_log_v3.csv, where every leader_cmd was frozen for ~4s).
"""
import argparse
import time

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM8")
ap.add_argument("--id", default="twin_leader_2")
args = ap.parse_args()

from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

leader = SOLeader(SOLeaderTeleopConfig(port=args.port, id=args.id, use_degrees=False))
leader.connect()
print("Connected. Move the leader arm now -- reading for 10s.\n")
try:
    t0 = time.time()
    while time.time() - t0 < 10:
        action = leader.get_action()
        print({k: round(v, 1) for k, v in action.items()})
        time.sleep(0.3)
finally:
    leader.disconnect()
    print("Disconnected.")
