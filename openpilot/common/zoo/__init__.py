"""Shared helpers for the zoo private S3 archive."""


def zoo_privacy_mode(params) -> bool:
  """True when the private S3 archive is on and the comma link is off.

  The one definition of privacy mode: no comma registration, athena,
  Sentry, or sunnylink traffic. Manager and process_config must agree
  on exactly this predicate.
  """
  return params.get_bool("ZooS3Enabled") and not params.get_bool("ZooS3CommaLinkEnabled")
