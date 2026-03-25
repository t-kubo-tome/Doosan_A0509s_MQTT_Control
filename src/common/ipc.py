
from multiprocessing.managers import DictProxy, SyncManager
import threading


class TopicMemory:
    """トピックのプロセス間共有メモリ
    
    トピックの種類（実際のトピック名とは違っても良い）は事前に固定する必要がある
    """
    def __init__(self, manager: SyncManager, topic_types: list[str]) -> None:
        # Manager経由のプロキシオブジェクトなので、これをサブプロセスに引数として渡すことで
        # コピーされるが、それでも同じ共有メモリを指すことになる
        self._store: DictProxy = manager.dict()
        self._locks: dict[str, threading.Lock] = {
            topic: threading.Lock() for topic in topic_types
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
        # with self._locks[topic_type]:
        # コピーを返す
        return dict(self._store[topic_type])

    def read_all(self) -> dict:
        """全トピックの種類のスナップショットを取得"""
        return {topic_type: self.read(topic_type) for topic_type in self._locks}

    def topic_types(self) -> list[str]:
        """全トピックの種類を取得"""
        return list(self._locks.keys())


if __name__ == "__main__":
    def test_topic_memory():
        from multiprocessing import Manager, Process

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
