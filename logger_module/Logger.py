import os
import logging
import logging.config

class ConfigurableLogger:
    def __init__(self, config_path=None, log_dir=None):
        self.config_path = config_path
        self.log_dir = log_dir

        logging.config.fileConfig(config_path)

        self.logger = logging.getLogger()
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        for handler in self.logger.handlers:
            if isinstance(handler, logging.FileHandler):
                log_filename = os.path.basename(handler.baseFilename)
                new_log_file = os.path.join(log_dir, log_filename)
                handler.close()
                self.logger.removeHandler(handler)
                new_handler = logging.FileHandler(new_log_file, mode='a')
                new_handler.setFormatter(handler.formatter)  # 保持原有的日志格式
                new_handler.setLevel(handler.level)  # 保持原有的日志级别
                self.logger.addHandler(new_handler)

    def debug(self, message):
        """记录DEBUG级别的日志"""
        self.logger.debug(message)

    def info(self, message):
        """记录INFO级别的日志"""
        self.logger.info(message)

    def warning(self, message):
        """记录WARNING级别的日志"""
        self.logger.warning(message)

    def error(self, message):
        """记录ERROR级别的日志"""
        self.logger.error(message)

    def critical(self, message):
        """记录CRITICAL级别的日志"""
        self.logger.critical(message)


# 使用示例
if __name__ == "__main__":
    logger = ConfigurableLogger('./logger_module/logging.conf', log_dir='./logs')

    # 记录不同级别的日志
    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
    logger.critical("This is a critical message")
