import logging

class NoDiffIKWarnings(logging.Filter):
    def filter(self, record):
        return not record.getMessage().startswith('Differential IK')

class NoSDFWarnings(logging.Filter):
    def filter(self, record):
        return 'Ignoring unsupported SDFormat' not in record.getMessage()