import json
import os

class ConfigurationHelper:
    configuration = None

    @staticmethod
    def getConfiguration():
        if ConfigurationHelper.configuration is not None:
            return ConfigurationHelper.configuration
        
        configurationFilePath = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "configuration",
            "configuration.json")
        
        with open(configurationFilePath, 'r') as f:
            configuration = json.loads(f.read())

        ConfigurationHelper.configuration = configuration

        return configuration