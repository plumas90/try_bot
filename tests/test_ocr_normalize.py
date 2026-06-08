import sys
sys.path.insert(0, 'src/capstone_ocr')

from capstone_ocr.ocr_reader_node import OcrReaderNode

import unittest
from unittest.mock import patch, MagicMock

class TestNormalizeText(unittest.TestCase):
    def setUp(self):
        with patch('rclpy.node.Node.__init__', return_value=None), \
             patch.object(OcrReaderNode, 'declare_parameter'), \
             patch.object(OcrReaderNode, 'get_parameter', return_value=MagicMock(value='/image_raw')), \
             patch.object(OcrReaderNode, 'create_subscription'), \
             patch.object(OcrReaderNode, 'create_publisher'), \
             patch('easyocr.Reader'):
            self.node = OcrReaderNode.__new__(OcrReaderNode)
            self.node.get_logger = MagicMock(return_value=MagicMock())

    def test_exact_match(self):
        assert self.node.normalize_text('LAPTOP') == 'LAPTOP'
        assert self.node.normalize_text('CHAIR') == 'CHAIR'
        assert self.node.normalize_text('BACKPACK') == 'BACKPACK'

    def test_lowercase_input(self):
        assert self.node.normalize_text('laptop') == 'LAPTOP'

    def test_fuzzy_match(self):
        assert self.node.normalize_text('LOPTOP') == 'LAPTOP'
        assert self.node.normalize_text('CHIAR') == 'CHAIR'

    def test_too_short_returns_empty(self):
        assert self.node.normalize_text('AB') == ''

    def test_digits_only_returns_empty(self):
        assert self.node.normalize_text('123') == ''

    def test_notebook_maps_to_laptop(self):
        assert self.node.normalize_text('NOTEBOOK') == 'LAPTOP'

if __name__ == '__main__':
    unittest.main()
