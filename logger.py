import logging
from logging.handlers import TimedRotatingFileHandler
import os

"""
工具类名称: logger.py

文件描述: 本文件是一个工具类，用于初始化日志配置并提供日志记录器。

公司: 上海具识智能科技有限公司

主要功能:
- 初始化日志配置，包括日志文件路径、日志格式、日志级别等。
- 提供获取日志记录器的方法，供其他模块使用。

作者: 王伟

版本: 1.0

完成日期: 2024年11月13日

"""


class LoggerUtility:
    def __init__(self, logger_name=__name__):
        """
        初始化LoggerUtility实例。

        参数:
        - logger_name (str): 日志记录器的名称，默认为调用模块的名称。
        """
        self.logger = logging.getLogger(logger_name)
        self._init_logger()

    def _init_logger(self):
        """
        私有方法，初始化日志配置。
        """
        log_directory = "log"
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)

        # 生成基础日志文件名
        log_filename = os.path.join(log_directory, "qrcode_service.log")

        # 创建 TimedRotatingFileHandler，日志每天轮转一次，保留30天的日志
        file_handler = TimedRotatingFileHandler(
            log_filename, when="midnight", interval=1, backupCount=30, encoding='utf-8')
        file_handler.suffix = "%Y-%m-%d"  # 设置轮转后日志文件名的后缀格式

        # 设置日志格式
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)

        # 控制台日志处理器
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)

        # 设置日志级别
        self.logger.setLevel(logging.INFO)

        # 清除已有的处理器，避免重复
        if self.logger.handlers:
            self.logger.handlers.clear()

        # 添加处理器
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

    def get_logger(self):
        """
        获取日志记录器。

        返回:
        - Logger对象
        """
        return self.logger


logger_util = LoggerUtility()
logger = logger_util.get_logger()

if __name__ == "__main__":
    logger.info("日志系统已初始化。")
    logger.debug("这是一个调试信息。")
    logger.warning("这是一个警告信息。")
    logger.error("这是一个错误信息。")
    logger.critical("这是一个严重错误信息。")
