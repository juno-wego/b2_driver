# Copyright (c) 2026 WeGo Robotics. All rights reserved.
# SPDX-License-Identifier: LicenseRef-Wego-Proprietary

"""Check Wego wrapper headers without assigning an open-source license."""

from pathlib import Path
import re
from xml.etree import ElementTree


def test_source_headers():
    """Preserve the package's proprietary SPDX identifier on maintained sources."""
    root = Path(__file__).resolve().parents[1]
    assert ElementTree.parse(root / 'package.xml').findtext('license') == 'Proprietary'
    files = [path for path in root.rglob('*') if path.is_file() and (
        path.suffix in {'.cpp', '.hpp', '.py', '.sh', '.cmake'} or
        path.name == 'CMakeLists.txt') and '__pycache__' not in path.parts]
    assert files
    for path in files:
        header = '\n'.join(path.read_text().splitlines()[:8])
        assert re.search(r'Copyright \(c\) \d{4} WeGo Robotics\.', header), str(path)
        assert 'SPDX-License-Identifier: LicenseRef-Wego-Proprietary' in header, str(path)
