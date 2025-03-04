import os
import time

from enum import StrEnum

class LogLevel(StrEnum):
    INFO = "    Info   "
    WARNING = "  Warning  "
    ERROR = "!!!ERROR!!!"
    DEBUG = "   Debug   "

class Logger:

    def __init__(self, console: bool, file: bool):
        
        self.console: bool = console
        self.file: bool = file

        self.dead = False

        if(self.file):
            os.makedirs("logs", exist_ok=True)
            self.location = f"logs/{time.asctime().replace(' ', '-')}"
            self.stream = open(self.location, "w", encoding="utf-8")

        self.log("   date   h:m:s:microsecond   source        level       message\n", info=False)

    def log(self, message: str, level: LogLevel = LogLevel.INFO, source: str = " GENERAL ", info: bool = True): 
        if(self.dead):
            self.stream = open(self.location, "w", encoding="utf-8")
            self.dead = False


        if(info):
            timestamp = time.time()
            seconds_timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))
            microseconds = int((timestamp * 1000000) % 1000000)

            formatted_timestamp = f"{seconds_timestamp}:{microseconds:06d}"

            message = formatted_timestamp + " : " + source + " : " + level + " : " + message

        if(level == LogLevel.ERROR):
            message = "\n!!!ERROR!!!\n" + message + "\n!!!ERROR!!!\n"


        if(self.console):
            print(message)

        if(self.file):
            self.stream.write(message + "\n")

    def end(self):
        if(self.file):
            self.stream.close()
            self.dead = True
            

    def create_sourced_logger(self, source):
        if(self.dead):
            self.stream = open(self.location, "w", encoding="utf-8")
            self.dead = False

        if len(source) <= 9:
            extra = 9 - len(source)
            back = extra // 2

            source = (" " * back) + source + (" " * (extra - back))

        return lambda message, level=LogLevel.INFO: self.log(message, level, source)
