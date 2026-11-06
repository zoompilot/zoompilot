"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from __future__ import annotations

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.models.fetcher import ModelCache, ModelFetcher

# stamped on the big-model catalog: it is jetlink's list, not sunnypilot's
JETLINK_KEY = "jetlink"


def _is_jetlinks(catalog) -> bool:
  return isinstance(catalog, dict) and catalog.get(JETLINK_KEY) is True


class BigCatalogCache(ModelCache):
  """The big-model catalog's cache, expired when it was fetched for other hardware: a chestnut
  coming or going changes whose list it is, and the cache would otherwise hide that for an hour.
  Once per change: offline, the refetch fails and the cache stands until it expires."""

  def __init__(self, params: Params, suffix: str, fetcher: ModelFetcherZP):
    super().__init__(params, suffix=suffix)
    self._refetched_for: bool | None = None
    self._fetcher = fetcher

  def get(self) -> tuple[dict, bool]:
    cached_data, is_expired = super().get()
    jetlink = jetlink_adapter.lists_big_models()
    self._fetcher.show_jetlink_list(jetlink)
    if cached_data and not is_expired and _is_jetlinks(cached_data) != jetlink and self._refetched_for != jetlink:
      self._refetched_for = jetlink
      cloudlog.warning(f"big-model catalog was fetched for {'a chestnut' if jetlink else 'jetlink'}; refetching")
      is_expired = True
    return cached_data, is_expired


class ModelFetcherZP(ModelFetcher):
  """sunnypilot's fetcher, with the big-model list jetlink's when no chestnut is fitted: the model
  index, every big model a Jetson can run, in its folders (jetlink_adapter.big_catalog). With a
  chestnut it is sunnypilot's own catalog, as fetched."""

  def __init__(self, params: Params):
    super().__init__(params)
    cache = self.model_caches["chestnut"] = BigCatalogCache(params, self.MODEL_SOURCES["chestnut"][1], self)
    # what the base just put
    self._active_json: dict[str, str] | None = {"qcom": self.MODEL_URL, "chestnut": self.MODEL_URL_CHESTNUT}
    self.show_jetlink_list(_is_jetlinks(params.get(cache._CACHE_KEY)))

  def show_jetlink_list(self, jetlink: bool) -> None:
    """ModelManager_ActiveJson without the chestnut catalog while jetlink's list stands in for it,
    so sunnylink lists the comma's cached copy rather than sunnypilot's."""
    active = {"qcom": self.MODEL_URL} if jetlink else {"qcom": self.MODEL_URL, "chestnut": self.MODEL_URL_CHESTNUT}
    if active != self._active_json:
      self.params.put("ModelManager_ActiveJson", active, block=True)
      self._active_json = active

  def _fetch_and_cache_models(self, source: str):
    if source != "chestnut" or not jetlink_adapter.lists_big_models():
      return super()._fetch_and_cache_models(source)
    from openpilot.sunnypilot.models.helpers import REQUIRED_JSON_VERSION
    catalog = jetlink_adapter.big_catalog(REQUIRED_JSON_VERSION)
    if catalog is None:  # offline, or no jetlink: the cache stands
      return None
    catalog = {**catalog, JETLINK_KEY: True}
    try:
      parsed = self.model_parser.parse_models(catalog)
    except Exception:
      cloudlog.exception("jetlink's big-model list does not parse")
      return None
    if parsed:
      self.model_caches[source].set(catalog)
    return parsed
