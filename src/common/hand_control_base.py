from abc import ABC, abstractmethod


class HandControlBase(ABC):
    """
    ハンド制御の抽象クラス.
    リアルタイム制御で使用しない関数は、デバッグしやすいため例外を送出する.
    リアルタイム制御で使用する関数は、例外処理のオーバーヘッドを避けるため、
    失敗した場合はFalseとエラーメッセージを返す.
    """
    def __init__(self, *args, **kwargs) -> None:
        pass

    @abstractmethod
    def connect_and_setup(self, **kwargs) -> None:
        """
        接続してハンドのパラメータを取得します.
        """
        pass

    @abstractmethod
    def disconnect(self, **kwargs) -> None:
        """
        接続を切断します.
        """
        pass

    @abstractmethod
    def grip(self, **kwargs) -> tuple[bool, str]:
        """
        グリップします.
        """
        pass
    
    @abstractmethod
    def release(self, **kwargs) -> tuple[bool, str]:
        """
        リリースします.
        """
        pass

    @abstractmethod
    def get_width(self, **kwargs) -> tuple[float, bool, str]:
        """
        ハンドの幅を取得します.
        """
        pass

    @abstractmethod
    def get_force(self, **kwargs) -> tuple[float, bool, str]:
        """
        ハンドの把持力を取得します.
        """
        pass


class DummyHandControl(HandControlBase):
    """ダミーのハンド制御クラス.通信接続が不要なツールを使用する場合に使用."""
    def __init__(self, **kwargs) -> None:
        self._conn = False
        self._width = 0
        self._force = 0

    def connect_and_setup(self, **kwargs) -> None:
        """
        接続してハンドのパラメータを取得します.
        """
        self._conn = True

    def disconnect(self, **kwargs) -> None:
        """
        接続を切断します.
        """
        if not self._conn:
            raise ValueError("Not connected")
        self._conn = False

    def grip(self, **kwargs) -> tuple[bool, str]:
        """
        グリップします.
        """
        if not self._conn:
            return False, "Not connected"
        self._width = 0
        self._force = 100
        return True, ""

    def release(self, **kwargs) -> tuple[bool, str]:
        """
        リリースします.
        """
        if not self._conn:
            return False, "Not connected"
        self._width = 100
        self._force = 0
        return True, ""

    def get_width(self, **kwargs) -> tuple[float, bool, str]:
        """
        ハンドの幅を取得します.
        """
        if not self._conn:
            return 0.0, False, "Not connected"
        return self._width, True, ""

    def get_force(self, **kwargs) -> tuple[float, bool, str]:
        """
        ハンドの把持力を取得します.
        """
        if not self._conn:
            return 0.0, False, "Not connected"
        return self._force, True, ""
