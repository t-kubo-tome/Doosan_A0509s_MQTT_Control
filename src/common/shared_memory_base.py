from multiprocessing import shared_memory

import numpy as np


class NamedSharedMemoryBase:
    """共有メモリアクセス用の基底クラス

    Args:
        create (bool): 共有メモリを初期化するかどうか。1つのプロセスだけでTrueにして
        multiprocessing.shared_memory.SharedMemoryを作成し、他のプロセスでは
        Falseにして既存のSharedMemoryにアクセスできるようにする
            
    派生クラスは共有メモリの名前nameと、配列の長さsizeを定義する必要がある

    派生クラスで共有メモリにアクセスするためのプロパティやセッターを定義して使用する    
    """
    name: str
    size: int

    def __init__(self, create: bool = False) -> None:
        # データ型はひな型で使用されていたfloat32を使用しているがfloat64でもいいはず
        dtype = np.dtype("float32")
        self._create = create
        sm_size = self.size * dtype.itemsize
        try:
            self._sm = shared_memory.SharedMemory(
                name=self.name, create=self._create, size=sm_size)
        except FileExistsError:
            self._sm = shared_memory.SharedMemory(
                name=self.name, create=False, size=sm_size)
        self._ar = np.ndarray((self.size,), dtype=dtype, buffer=self._sm.buf)
        if self._create:
            self._ar[:] = 0

    def release(self) -> None:
        """
        共有メモリを解放する
        共有メモリを使用する各プロセスの終了時にそれぞれ必ず呼び出すこと
        """
        self._sm.close()
        if self._create:
            self._sm.unlink()


if __name__ == "__main__":
    # multiprocessingをpytestでテストするのはできない?または煩雑?なので、
    # 簡単なテストコードをここに書いておく
    class DummyNamedSharedMemory(NamedSharedMemoryBase):
        name = "robot"
        size = 6

        @property
        def joint_states(self) -> np.ndarray:
            return self._ar[0:6]

        @joint_states.setter
        def joint_states(self, value: np.ndarray) -> None:
            self._ar[0:6] = value

    from multiprocessing import Process

    def test_named_shared_memory():
        shm1 = DummyNamedSharedMemory(create=True)
        assert np.array_equal(shm1.joint_states, np.zeros(6))
        shm1.joint_states = np.array([1, 2, 3, 4, 5, 6], dtype=np.float32)

        def sub_process():    
            shm2 = DummyNamedSharedMemory(create=False)
            assert np.array_equal(shm2.joint_states, np.array([1, 2, 3, 4, 5, 6], dtype=np.float32))
            shm2.release()
        
        p = Process(target=sub_process)
        p.start()
        p.join()
        
        shm1.release()

    test_named_shared_memory()
    print("Passed: test_named_shared_memory")
