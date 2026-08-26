import io
import json
import unittest
import urllib.error
import urllib.request

from stage2.qwen_http import open_json_with_retry


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class QwenHTTPRetryTest(unittest.TestCase):
    def setUp(self):
        self.request = urllib.request.Request("https://example.invalid", data=b"{}")

    def test_retries_transient_error_then_returns_json(self):
        attempts = []
        sleeps = []

        def opener(_request, timeout):
            attempts.append(timeout)
            if len(attempts) == 1:
                raise urllib.error.URLError("temporary DNS failure")
            return _Response(json.dumps({"ok": True}).encode())

        value, count = open_json_with_retry(
            self.request, timeout=9, retry_delays=(0.1,), opener=opener, sleeper=sleeps.append,
        )
        self.assertEqual(value, {"ok": True})
        self.assertEqual(count, 2)
        self.assertEqual(sleeps, [0.1])

    def test_does_not_retry_authentication_error(self):
        attempts = []

        def opener(request, timeout):
            attempts.append(timeout)
            raise urllib.error.HTTPError(
                request.full_url, 401, "unauthorized", {}, io.BytesIO(b"bad key")
            )

        with self.assertRaisesRegex(RuntimeError, "HTTP 401"):
            open_json_with_retry(
                self.request, timeout=9, retry_delays=(0.1, 0.2),
                opener=opener, sleeper=lambda _: None,
            )
        self.assertEqual(len(attempts), 1)


if __name__ == "__main__":
    unittest.main()
