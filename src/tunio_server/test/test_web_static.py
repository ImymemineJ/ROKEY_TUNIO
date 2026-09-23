"""복사/심볼릭 링크 설치에서 정적 파일 제공과 경로 이탈 차단을 검증한다."""
from functools import partial
from http.server import ThreadingHTTPServer
import threading
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

from tunio_server import web_node as W


@pytest.mark.parametrize('symlinks', [False, True])
def test_static_install_layout_and_path_guard(tmp_path, monkeypatch, symlinks):
    share = tmp_path / 'install' / 'share' / 'tunio_server'
    installed = share / 'web'
    source = tmp_path / 'src' / 'web'
    build = tmp_path / 'build' / 'web'
    installed.mkdir(parents=True)
    source.mkdir(parents=True)
    for name, content in [('index.html', '<html>TUNIO</html>'),
                          ('app.js', 'const tunio = true;'), ('style.css', 'body {}')]:
        (source / name).write_text(content)
        if symlinks:
            if not build.exists():
                build.parent.mkdir(parents=True)
                build.symlink_to(source, target_is_directory=True)
            (installed / name).symlink_to(build / name)
        else:
            (installed / name).write_text(content)
    monkeypatch.setattr(W, 'get_package_share_directory', lambda _: str(share))
    root = W._web_dir().resolve()
    assert root == (source if symlinks else installed)
    secret = tmp_path / 'secret.html'
    secret.write_text('not a public asset')
    (root / 'outside.html').symlink_to(secret)
    node = SimpleNamespace(get_logger=lambda: Mock())
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(W.Handler, node=node, web_dir=root))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        base = 'http://127.0.0.1:%d' % server.server_port
        for name in ('', 'index.html', 'app.js', 'style.css'):
            with urlopen(base + '/' + name, timeout=3) as response:
                assert response.status == 200
        for name in ('outside.html', '../secret.html', 'missing.html'):
            with pytest.raises(HTTPError) as exc:
                urlopen(base + '/' + name, timeout=3)
            assert exc.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        thread.join(3)
