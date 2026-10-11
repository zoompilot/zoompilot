import os
import shutil
import subprocess
import tempfile
import time

import pytest

from openpilot.common.zoo import pgp

GPG = 'gpg'


def short_gnupghome() -> str:
  # gpg-agent sockets fail on macOS when the homedir path is long (pytest's
  # tmp root is), so keyrings live under /tmp
  home = tempfile.mkdtemp(prefix='zoo-gpg-', dir='/tmp')
  os.chmod(home, 0o700)
  return home


def gpg_run(homedir: str, *args: str, check: bool = True) -> subprocess.CompletedProcess:
  proc = subprocess.run([GPG, '--homedir', homedir, '--batch', '--pinentry-mode', 'loopback', '--passphrase', '', *args], capture_output=True)
  if check and proc.returncode != 0:
    raise AssertionError(f'gpg {args} failed ({proc.returncode}): {proc.stderr.decode()}')
  return proc


def make_keyring(homedir: str, subkey_algo: str, expire: str = 'never', uid: str = 'zoo archive') -> str:
  gpg_run(homedir, '--quick-generate-key', uid, 'ed25519', 'cert', 'never')
  fpr_out = gpg_run(homedir, '--list-keys', '--with-colons').stdout.decode()
  fprs = [line.split(':')[9] for line in fpr_out.splitlines() if line.startswith('fpr')]
  # keys list oldest first: the key just created is the last one
  fpr = fprs[-1]
  gpg_run(homedir, '--quick-add-key', fpr, subkey_algo, 'encr', expire)
  return gpg_run(homedir, '--armor', '--export', fpr).stdout.decode()


def cleanup_gnupghome(home: str) -> None:
  # a lingering gpg-agent from a previous run makes the next run's keygen
  # fail with "No agent running"; kill it before removing the keyring
  subprocess.run(['gpgconf', '--homedir', home, '--kill', 'gpg-agent'], capture_output=True)
  shutil.rmtree(home, ignore_errors=True)


@pytest.fixture(scope='module')
def gnupghome():
  if subprocess.run([GPG, '--version'], capture_output=True).returncode != 0:
    pytest.skip('gpg not installed')
  home = short_gnupghome()
  yield home
  cleanup_gnupghome(home)


@pytest.fixture(scope='module')
def cv25519_block(gnupghome):
  return make_keyring(gnupghome, 'cv25519', uid='zoo cv')


@pytest.fixture(scope='module')
def rsa3072_block(gnupghome):
  return make_keyring(gnupghome, 'rsa3072', uid='zoo rsa')


@pytest.fixture(scope='module')
def expired_block():
  if subprocess.run([GPG, '--version'], capture_output=True).returncode != 0:
    pytest.skip('gpg not installed')
  home = short_gnupghome()
  block = make_keyring(home, 'cv25519', expire='seconds=1', uid='zoo expired')
  time.sleep(2)
  shutil.rmtree(home, ignore_errors=True)
  return block


def gpg_decrypt(gnupghome, message: bytes) -> bytes:
  with open('/tmp/zoo_pgp_msg.bin', 'wb') as f:
    f.write(message)
  out = gpg_run(gnupghome, '-d', '/tmp/zoo_pgp_msg.bin')
  return out.stdout


SIZES = [0, 1, 15, 16, 17, 18, 19, 20, 32, 511, 4096, 9000, 100000]


@pytest.mark.parametrize('size', SIZES)
def test_cv25519_round_trip(gnupghome, cv25519_block, size):
  recipients = pgp.load_recipients(cv25519_block)
  assert recipients[0]['key'].algo == pgp.ECDH_ALGO
  data = os.urandom(size)
  assert gpg_decrypt(gnupghome, pgp.encrypt(data, recipients)) == data


@pytest.mark.parametrize('size', SIZES)
def test_rsa3072_round_trip(gnupghome, rsa3072_block, size):
  recipients = pgp.load_recipients(rsa3072_block)
  assert recipients[0]['key'].algo in pgp.RSA_ENCRYPT_ALGOS
  data = os.urandom(size)
  assert gpg_decrypt(gnupghome, pgp.encrypt(data, recipients)) == data


def test_multi_recipient(gnupghome, cv25519_block, rsa3072_block):
  recipients = pgp.load_recipients(cv25519_block) + pgp.load_recipients(rsa3072_block)
  data = b'multi recipient payload'
  assert gpg_decrypt(gnupghome, pgp.encrypt(data, recipients)) == data


def test_armored_round_trip(gnupghome, cv25519_block):
  recipients = pgp.load_recipients(cv25519_block)
  data = b'armored payload'
  with open('/tmp/zoo_pgp_msg.asc', 'w') as f:
    f.write(pgp.armor(pgp.encrypt(data, recipients)))
  assert gpg_run(gnupghome, '-d', '/tmp/zoo_pgp_msg.asc').stdout == data


def test_armor_wraps_long_payload(gnupghome, cv25519_block):
  # several base64 lines: the 64-char wrap and the CRC24 ride a real payload
  data = os.urandom(3000)
  recipients = pgp.load_recipients(cv25519_block)
  armored = pgp.armor(pgp.encrypt(data, recipients))
  body = [ln for ln in armored.splitlines() if ln and not ln.startswith(('=', '-'))]
  assert len(body) > 1 and all(len(ln) <= 64 for ln in body)
  with open('/tmp/zoo_pgp_msg.asc', 'w') as f:
    f.write(armored)
  assert gpg_run(gnupghome, '-d', '/tmp/zoo_pgp_msg.asc').stdout == data


def test_encrypt_file_round_trip(gnupghome, cv25519_block):
  recipients = pgp.load_recipients(cv25519_block)
  data = os.urandom(3 * 1024 * 1024 + 7)  # crosses many chunk and CFB boundaries
  with open('/tmp/zoo_pgp_src.bin', 'wb') as f:
    f.write(data)
  pgp.encrypt_file('/tmp/zoo_pgp_src.bin', '/tmp/zoo_pgp_src.gpg', recipients, chunk_size=1 << 16)
  assert gpg_run(gnupghome, '-d', '/tmp/zoo_pgp_src.gpg').stdout == data


def test_expired_key_strict_fails(expired_block):
  with pytest.raises(pgp.PgpError, match='expired'):
    pgp.load_recipients(expired_block, strict_expiry=True)


def test_expired_key_runtime_allows(expired_block):
  recipients = pgp.load_recipients(expired_block, strict_expiry=False)
  assert len(recipients) == 1


def test_sign_only_keyring_rejected(gnupghome):
  gpg_run(gnupghome, '--quick-generate-key', 'zoo sign only', 'ed25519', 'sign', 'never')
  export = gpg_run(gnupghome, '--armor', '--export', 'zoo sign only').stdout.decode()
  with pytest.raises(pgp.PgpError, match='no usable encryption key'):
    pgp.load_recipients(export)


def test_skip_unusable_subkey_still_works(gnupghome, cv25519_block):
  # an extra unsupported subkey next to the usable cv25519 one must not break the block
  fpr_out = gpg_run(gnupghome, '--list-keys', '--with-colons').stdout.decode()
  fprs = [line.split(':')[9] for line in fpr_out.splitlines() if line.startswith('fpr')]
  gpg_run(gnupghome, '--quick-add-key', fprs[0], 'ed25519', 'sign', 'never')
  export = gpg_run(gnupghome, '--armor', '--export', fprs[0]).stdout.decode()
  recipients = pgp.load_recipients(export)
  data = b'payload after adding a sign-only subkey'
  assert gpg_decrypt(gnupghome, pgp.encrypt(data, recipients)) == data
