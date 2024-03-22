import json
import os

class ConfigurationHelper:
    __configuration = None

    @staticmethod
    def getConfiguration():
        if ConfigurationHelper.__configuration is not None:
            return ConfigurationHelper.__configuration
        
        configurationFilePath = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "configuration",
            "configuration.json")
        
        with open(configurationFilePath, 'r') as f:
            configuration = json.loads(f.read())

        ConfigurationHelper.__configuration = configuration

        return configuration