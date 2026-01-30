import datetime
import logging


class MicrosecondFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        dt = datetime.datetime.fromtimestamp(record.created)
        if datefmt:
            s = dt.strftime(datefmt)
            # %f をマイクロ秒で置換
            s = s.replace('%f', f"{dt.microsecond:06d}")
            return s
        else:
            return super().formatTime(record, datefmt)
