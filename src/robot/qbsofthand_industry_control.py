from ..common.hand_control_base import HandControlBase
from .qbsofthand_industry_api_pybind import qbSoftHandIndustryAPI


class QbSoftHandIndustryControl(HandControlBase):
    """qbSoftHandIndustryのハンド制御クラス."""
    def __init__(self, hand_ip: str, max_timeout: int = 10) -> None:
        self._hand_ip = hand_ip
        # デフォルトは実験による推奨値
        self._max_timeout = max_timeout
        self._qb_hand: qbSoftHandIndustryAPI | None = None

    def connect_and_setup(self, **kwargs) -> None:
        """
        接続してハンドのパラメータを取得します.
        """
        self._qb_hand = qbSoftHandIndustryAPI(self._hand_ip, self._max_timeout)
        if not self._qb_hand.isInitialized():
            raise ValueError("Failed to initialize qbSoftHandIndustryAPI")

    def disconnect(self, **kwargs) -> None:
        """
        接続を切断します.
        """
        if self._qb_hand is None:
            raise ValueError("qbSoftHandIndustryAPI is not initialized")
        self._qb_hand = None

    def grip(
        self,
        position: float = 100,
        velocity: float = 50,
        force: float = 62.5,
    ) -> tuple[bool, str]:
        """
        グリップします.
        Default: fully close the hand at half speed and minimum applied force
        (62.5% of max force is the minimum value that can be set)
        """
        if self._qb_hand is None:
            return False, "qbSoftHandIndustryAPI is not initialized"
        ret = self._qb_hand.setClosure(position, velocity, force)
        # Success
        if ret == 0:
            return True, ""
        elif ret == -1:
            return False, "the position is out of range"
        elif ret == -3:
            return False, "communication is lost"
        else:
            return False, f"unknown error, return code: {ret}"

    def release(
        self,
        position: float = 0,
        velocity: float = 100,
        force: float = 100,
    ) -> tuple[bool, str]:
        """
        リリースします.
        Default: reopen at full speed and full force
        """
        if self._qb_hand is None:
            return False, "qbSoftHandIndustryAPI is not initialized"
        ret = self._qb_hand.setClosure(position, velocity, force)
        # Success
        if ret == 0:
            return True, ""
        elif ret == -1:
            return False, "the position is out of range"
        elif ret == -3:
            return False, "communication is lost"
        else:
            return False, f"unknown error, return code: {ret}"

    def get_width(self, **kwargs) -> tuple[float | None, bool, str]:
        """
        ハンドの幅を取得します.(%単位)
        """
        if self._qb_hand is None:
            return None, False, "qbSoftHandIndustryAPI is not initialized"
        return self._qb_hand.getPosition(), True, ""

    def get_force(self, **kwargs) -> tuple[float | None, bool, str]:
        """
        ハンドの把持力を取得します.(%単位)
        """
        if self._qb_hand is None:
            return None, False, "qbSoftHandIndustryAPI is not initialized"
        return self._qb_hand.getCurrent(), True, ""
