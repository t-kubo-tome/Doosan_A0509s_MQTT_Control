from multiprocessing import shared_memory
from multiprocessing.managers import DictProxy, SyncManager
import threading

import numpy as np


class TopicMemory:
    """トピックのプロセス間共有メモリ
    
    トピックの種類（実際のトピック名とは違っても良い）は事前に固定する必要がある
    """
    def __init__(self, manager: SyncManager, topic_types: list[str]) -> None:
        # Manager経由のプロキシオブジェクトなので、これをサブプロセスに引数として渡すことで
        # コピーされるが、それでも同じ共有メモリを指すことになる
        self._store: DictProxy = manager.dict()
        self._locks: dict[str, threading.Lock] = {
            topic: manager.Lock() for topic in topic_types
        }
        for topic in topic_types:
            self._store[topic] = {}

    def write(self, topic_type: str, data: dict) -> None:
        """指定したトピックの種類にデータを書き込む(Lock付き)"""
        if topic_type not in self._locks:
            raise KeyError(f"未登録のトピックの種類: {topic_type}")
        with self._locks[topic_type]:
            self._store[topic_type] = data

    def read(self, topic_type: str) -> dict:
        """指定したトピックの種類のデータを読み取る(Lock付き)"""
        if topic_type not in self._locks:
            raise KeyError(f"未登録のトピックの種類: {topic_type}")
        with self._locks[topic_type]:
            # コピーを返す
            return dict(self._store[topic_type])

    def read_all(self) -> dict:
        """全トピックの種類のスナップショットを取得"""
        return {topic_type: self.read(topic_type) for topic_type in self._locks}

    def topic_types(self) -> list[str]:
        """全トピックの種類を取得"""
        return list(self._locks.keys())


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
        self._sm = shared_memory.SharedMemory(
            name=self.name, create=self._create, size=sm_size)
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

    def test_topic_memory():
        from multiprocessing import Manager

        manager = Manager()
        topic_memory = TopicMemory(manager, topic_types=["topic1", "topic2"])
        topic_memory.write("topic1", {"key": "value"})
        assert topic_memory.read("topic1") == {"key": "value"}
        assert topic_memory.read("topic2") == {}

        def sub_process(topic_memory: TopicMemory):
            assert topic_memory.read("topic1") == {"key": "value"}
            topic_memory.write("topic2", {"another_key": 123})

        p = Process(target=sub_process, args=(topic_memory,))
        p.start()
        p.join()

        assert topic_memory.read_all() == {"topic1": {"key": "value"}, "topic2": {"another_key": 123}}
        assert topic_memory.topic_types() == ["topic1", "topic2"]
    
    test_topic_memory()
    print("Passed: test_topic_memory")
