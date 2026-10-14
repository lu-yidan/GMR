"""Fetch pinned official Three.js assets and verify npm SHA-512 integrity."""
import base64
import hashlib
import io
import json
from pathlib import Path
import tarfile
from urllib.request import urlopen

VERSION = '0.180.0'
DEST = Path(__file__).resolve().parents[1] / 'motion_data/duobox_web/vendor'


def main():
    with urlopen(f'https://registry.npmjs.org/three/{VERSION}', timeout=60) as response:
        meta = json.load(response)
    with urlopen(meta['dist']['tarball'], timeout=120) as response:
        blob = response.read()
    actual = 'sha512-' + base64.b64encode(hashlib.sha512(blob).digest()).decode()
    if actual != meta['dist']['integrity']:
        raise RuntimeError('Three.js integrity verification failed')
    DEST.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(blob)) as archive:
        for source, target in [('package/build/three.module.js', 'three.module.js'),
                               ('package/build/three.core.js', 'three.core.js'),
                               ('package/examples/jsm/controls/OrbitControls.js', 'OrbitControls.js'),
                               ('package/LICENSE', 'LICENSE')]:
            (DEST / target).write_bytes(archive.extractfile(source).read())
    print(f'Three.js {VERSION} verified and installed in {DEST}')


if __name__ == '__main__':
    main()
