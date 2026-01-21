import sys
from pathlib import Path

src_path = Path(__file__).parent.parent
sys.path.insert(0, str(src_path))

import time

from doosan.qbsofthand_industry_api_pybind import qbSoftHandIndustryAPI


if __name__ == "__main__":
    # device_ip = "192.168.207.44"
    device_ip = "192.168.5.44"
    max_timeout = 30  # seconds
    hand = qbSoftHandIndustryAPI(device_ip, max_timeout)
    print(f"{hand.isInitialized()=}")
    print(f"{hand.getStatistics()=}")
    print(f"{hand.getCurrent()=}")
    print(f"{hand.getVelocity()=}")
    print(f"{hand.getPosition()=}")
    # fully close the hand at half speed and minimum applied force (62.5% of max force is the minimum value that can be set)
    print(f"{hand.setClosure(100, 50, 62.5)=}")
    print(f"{hand.waitForTargetReached()=}")
    print(f"{hand.getPosition()=}")
    # reopen at full speed and full force
    print(f"{hand.setClosure(0, 100, 100)=}")
    print(f"{hand.waitForTargetReached()=}")
    print(f"{hand.getPosition()=}")
    # IP is set only after the power is restarted
    net_ip = "192.168.5.44"
    net_mask = "255.255.255.0"
    net_gateway = "192.168.5.1"
    print(f"{hand.setIP(net_ip, net_mask, net_gateway)=}")
    time.sleep(3)
    print(f"{hand.isInitialized()=}")
    print(f"{hand.getStatistics()=}")
    print(f"{hand.getPosition()=}")
