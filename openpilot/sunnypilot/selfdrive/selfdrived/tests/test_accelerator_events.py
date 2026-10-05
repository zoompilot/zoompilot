from types import SimpleNamespace

from openpilot.cereal import custom, messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import EVENT_NAME, EVENTS, Events, EventName, ET
from openpilot.selfdrive.selfdrived.state import State, StateMachine
from openpilot.sunnypilot.mads.state import StateMachine as MadsStateMachine, State as MadsState
from openpilot.sunnypilot.selfdrive.selfdrived.accelerator_events import (AcceleratorEvents, HANDBACK_TICKS,
                                                                           SWITCHING_TICKS)
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENT_NAME_SP, EVENTS_SP, EventsSP

EventNameSP = custom.OnroadEventSP.EventName


class AcceleratorEventsTest(OpenpilotTestCase):
  """The adapter alone; the drives through SelfdriveD are traced in
  test_selfdrived_traces.py beside this one."""

  def setUp(self):
    super().setUp()
    self.sm = messaging.SubMaster(['modelV2', 'modelDataV2SP'])
    self.events = Events()
    self.events_sp = EventsSP()
    self.accel = AcceleratorEvents()
    for service in self.sm.services:
      self.sm.data[service] = self.sm[service].as_builder()
      self.sm.seen[service] = self.sm.alive[service] = self.sm.valid[service] = True

  def step(self, state='none', big=False, alive=True, enabled=False, mads=False) -> set[str]:
    """One selfdrived tick, `mads` being MADS engaged; what the adapter raised,
    native and sunnypilot, by name."""
    self.sm['modelDataV2SP'].acceleratorState = state
    self.sm['modelV2'].big = big
    self.sm.alive['modelV2'] = alive
    self.events.clear()
    self.events_sp.clear()
    self.accel.update(self.sm, enabled or mads, self.events, self.events_sp)
    return {EVENT_NAME[e] for e in self.events.names} | {EVENT_NAME_SP[e] for e in self.events_sp.names}

  def drive_big(self, **kwargs) -> None:
    """The large model swapped in and past its first second."""
    for _ in range(SWITCHING_TICKS + 1):
      self.step(state='running', big=True, **kwargs)

  def ticks_with(self, event: str, **kwargs) -> int:
    """How many ticks in a row `event` is raised from here on, up to a minute."""
    for n in range(6000):
      if event not in self.step(**kwargs):
        return n
    return 6000


class TestSettling(AcceleratorEventsTest):
  """A switch either way is settling for a second: selfdrived holds the comm
  and localization errors the switch's dropped frames would raise."""

  def test_a_swap_in_settles_for_a_second(self):
    self.step()
    self.assertFalse(self.accel.settling)
    settled = [(self.step(state='running', big=True), self.accel.settling)[1] for _ in range(SWITCHING_TICKS + 5)]
    self.assertEqual(settled, [True] * SWITCHING_TICKS + [False] * 5)

  def test_a_hand_back_settles_for_a_second(self):
    self.drive_big()
    self.assertFalse(self.accel.settling)
    settled = [(self.step(state='retrying', big=False), self.accel.settling)[1] for _ in range(SWITCHING_TICKS + 5)]
    self.assertEqual(settled, [True] * SWITCHING_TICKS + [False] * 5)

  def test_settling_is_what_selfdrived_holds_the_errors_on(self):
    # the gate reads the property, not the switching chime's own counter
    from openpilot.selfdrive.selfdrived import selfdrived
    import inspect
    self.assertIn('self.accelerator_events.settling', inspect.getsource(selfdrived.SelfdriveD.update_events))


class TestHandBack(AcceleratorEventsTest):
  def test_event_ordinals(self):
    # logs store the ordinal, and 28-31 shipped before these existed
    self.assertEqual(int(EventNameSP.stockEcuReady), 31)
    self.assertEqual(int(EventNameSP.bigModelAvailable), 32)
    self.assertEqual(int(EventNameSP.bigModelLinkLost), 33)

  def test_a_fall_while_engaged_warns_for_five_seconds(self):
    self.drive_big(enabled=True)
    # the tick of the fall is the first of them
    self.assertEqual(self.step(state='retrying', enabled=True), {'bigModelLinkLost'})
    self.assertEqual(1 + self.ticks_with('bigModelLinkLost', state='retrying', enabled=True), HANDBACK_TICKS)
    self.assertEqual(HANDBACK_TICKS, 500)

  def test_a_fall_with_mads_lateral_paused_warns_once_it_steers(self):
    # paused (a stop, the brake) is still engaged: MADS steers again on its own
    self.drive_big(mads=True)
    self.assertEqual(self.step(state='retrying', mads=True), {'bigModelLinkLost'})

  def test_a_mads_only_fall_warns(self):
    # lateral only, cruise off: the 2026-09-29 drives lost the link three
    # times like this with no alert at all
    self.drive_big(mads=True)
    self.assertEqual(self.step(state='retrying', mads=True), {'bigModelLinkLost'})
    self.assertEqual(1 + self.ticks_with('bigModelLinkLost', state='retrying', mads=True), HANDBACK_TICKS)

  def test_a_disengage_ends_it_and_a_reengage_does_not_bring_it_back(self):
    self.drive_big(enabled=True)
    for _ in range(100):
      self.assertEqual(self.step(state='retrying', enabled=True), {'bigModelLinkLost'})
    self.assertEqual(self.step(state='retrying'), set())
    self.assertEqual(self.step(state='retrying', enabled=True), set())

  def test_a_fall_while_disengaged_is_silent(self):
    self.drive_big()
    self.assertEqual(self.step(state='retrying'), set())
    # the fall was consumed: engaging afterwards does not replay it
    self.assertEqual(self.step(state='retrying', enabled=True), set())

  def test_a_chestnut_fall_is_not_the_adapters(self):
    # comma's board has the native bigModelFailed for this, and nothing here adds to it
    self.step(state='none', big=True, enabled=True)
    self.assertEqual(self.step(state='none', enabled=True), set())

  def test_the_adapter_never_raises_big_model_failed(self):
    # upstream's text for it says to restart the car, which the 2026-09-29
    # drive showed after a Mac was unplugged while engaged
    self.drive_big(enabled=True, mads=True)
    for state in ('retrying', 'unavailable', 'joining', 'ready', 'none'):
      self.assertNotIn('bigModelFailed', self.step(state=state, enabled=True, mads=True))
      self.assertNotIn('bigModelFailed', self.step(state=state, alive=False, enabled=True, mads=True))

  def test_link_lost_is_a_warning_and_nothing_else(self):
    alerts = EVENTS_SP[EventNameSP.bigModelLinkLost]
    self.assertEqual(set(alerts), {ET.WARNING})
    alert = alerts[ET.WARNING]
    self.assertEqual((alert.alert_text_1, alert.alert_text_2), ("Big Model Lost", "Using small model"))


class TestHandBackOnTheRealStateMachines(AcceleratorEventsTest):
  """The warning through selfdrived's own machinery: the main state machine,
  MADS's, and the alert manager, in the order SelfdriveD.step runs them."""

  def drive(self, ticks, enabled=True, mads=False, disengage_at=None):
    """Engaged on the big model, then the link falls; the main and MADS states
    at the end, and the alert on screen at every tick from the fall."""
    main = StateMachine()
    main.state = State.enabled if enabled else State.disabled
    selfdrive = SimpleNamespace(state_machine=main, events=self.events, events_sp=self.events_sp, enabled=enabled)
    madsm = MadsStateMachine(SimpleNamespace(selfdrive=selfdrive, button_owns_lateral=False))
    madsm.state = MadsState.enabled if mads else MadsState.disabled
    am = AlertManager()
    self.drive_big(enabled=enabled, mads=mads)
    shown = []
    for frame in range(ticks):
      if frame == disengage_at:
        main.state, madsm.state = State.disabled, MadsState.disabled
        selfdrive.enabled = enabled = mads = False
      self.step(state='retrying', enabled=enabled, mads=mads)
      enabled, _ = main.update(self.events)
      selfdrive.enabled = enabled
      mads, _ = madsm.update()
      clear = set() if ET.WARNING in main.current_alert_types else {ET.WARNING}
      am.add_many(frame, self.events.create_alerts(main.current_alert_types, []) +
                  self.events_sp.create_alerts(main.current_alert_types, []))
      am.process_alerts(frame, clear)
      shown.append(am.current_alert.alert_text_1)
    return main.state, madsm.state, shown

  def test_engaged_it_stays_engaged_and_warns_for_five_seconds(self):
    main, madsm, shown = self.drive(HANDBACK_TICKS + 100, mads=True)
    self.assertEqual((main, madsm), (State.enabled, MadsState.enabled))
    self.assertEqual(shown[:HANDBACK_TICKS], ["Big Model Lost"] * HANDBACK_TICKS)
    self.assertEqual(set(shown[HANDBACK_TICKS + 1:]), {""})

  def test_mads_only_it_keeps_steering_and_warns(self):
    main, madsm, shown = self.drive(HANDBACK_TICKS + 100, enabled=False, mads=True)
    self.assertEqual((main, madsm), (State.disabled, MadsState.enabled))
    self.assertEqual(shown[:HANDBACK_TICKS], ["Big Model Lost"] * HANDBACK_TICKS)
    self.assertEqual(set(shown[HANDBACK_TICKS + 1:]), {""})

  def test_a_disengage_clears_it_at_once(self):
    _, _, shown = self.drive(300, mads=True, disengage_at=100)
    self.assertEqual(set(shown[:100]), {"Big Model Lost"})
    self.assertEqual(set(shown[100:]), {""})


class TestSwitching(AcceleratorEventsTest):
  """For a second after a swap nothing engages: the large model starts from an
  empty history. Upstream's bigModelLoading no-entry, which both state
  machines read."""

  def test_a_second_of_no_entry_after_a_swap_then_the_chime(self):
    self.step(state='ready')
    self.assertEqual(self.step(state='running', big=True), {'bigModelLoading'})
    self.assertEqual(1 + self.ticks_with('bigModelLoading', state='running', big=True), SWITCHING_TICKS)
    self.assertEqual(SWITCHING_TICKS, 100)
    self.assertIn(ET.NO_ENTRY, EVENTS[EventName.bigModelLoading])

  def test_the_chime_waits_for_the_end_of_the_second(self):
    # selfdrived's native block raises it on the swap; it now says "engage now"
    self.step(state='ready')
    heard = []
    for tick in range(SWITCHING_TICKS + 10):
      self.events.clear()
      self.events_sp.clear()
      if tick == 0:
        self.events_sp.add(EventNameSP.bigModelReady)   # the native block, on modelV2.big's edge
      self.sm['modelDataV2SP'].acceleratorState = 'running'
      self.sm['modelV2'].big = True
      self.accel.update(self.sm, False, self.events, self.events_sp)
      heard.append(EventNameSP.bigModelReady in self.events_sp.names)
    self.assertEqual([i for i, h in enumerate(heard) if h], [SWITCHING_TICKS])

  def test_a_fall_inside_the_second_ends_it_without_the_chime(self):
    self.step(state='ready')
    for _ in range(10):
      self.step(state='running', big=True)
    self.assertEqual(self.step(state='retrying'), set())
    for _ in range(SWITCHING_TICKS):
      self.assertEqual(self.step(state='retrying'), set())

  def drive_presses(self, press_at: int, ticks: int = SWITCHING_TICKS + 40):
    """A swap at tick 0, then one press of cruise and one of MADS at `press_at`,
    as a car sends them: an edge, never repeated. What paused MADS resumes on,
    silentLkasEnable, is raised while it is paused, as mads.update_events does
    with no brake held. Returns (main, MADS) engaged at every tick."""
    main = StateMachine()
    selfdrive = SimpleNamespace(state_machine=main, events=self.events, events_sp=self.events_sp, enabled=False,
                                model_startup=SimpleNamespace(starting=False), big_model_loading=False)
    madsm = MadsStateMachine(SimpleNamespace(selfdrive=selfdrive, button_owns_lateral=False))
    self.step(state='ready')
    engaged = []
    for tick in range(ticks):
      self.step(state='running', big=True, enabled=selfdrive.enabled, mads=madsm.state != MadsState.disabled)
      if tick == press_at:
        self.events.add(EventName.buttonEnable)
        self.events_sp.add(EventNameSP.lkasEnable)
      if madsm.state == MadsState.paused:
        self.events_sp.add(EventNameSP.silentLkasEnable)
      enabled, _ = main.update(self.events)
      selfdrive.enabled = enabled
      _, mads_active = madsm.update()
      engaged.append((enabled, mads_active, madsm.state))
    return engaged

  def test_one_press_inside_the_second_is_not_lost(self):
    # 2026-09-29: main went on 0.27 s after a swap. MADS's enable is the main-on
    # edge; refused, it would have left MADS off with main on
    engaged = self.drive_presses(press_at=27)
    self.assertEqual(engaged[27][:2], (False, False))
    self.assertEqual(engaged[27][2], MadsState.paused)
    self.assertTrue(all(state == MadsState.paused for _, _, state in engaged[27:SWITCHING_TICKS]))
    # MADS steers from the end of the second, with no second press; cruise was
    # refused, as a no-entry does, and waits for the driver's next press
    self.assertEqual(engaged[SWITCHING_TICKS][:2], (False, True))
    self.assertEqual(engaged[-1][:2], (False, True))

  def test_one_press_after_the_second_engages_both(self):
    engaged = self.drive_presses(press_at=SWITCHING_TICKS + 5)
    self.assertEqual(engaged[SWITCHING_TICKS + 4][:2], (False, False))
    self.assertEqual(engaged[SWITCHING_TICKS + 5][:2], (True, True))

  def test_a_chestnut_never_switches_like_this(self):
    self.step(state='none')
    self.assertEqual(self.step(state='none', big=True), set())


class TestOptionalProcesses(OpenpilotTestCase):
  def test_a_dead_link_owner_never_blocks_engagement(self):
    # manager does not restart a process that died, and selfdrived's
    # processNotRunning is NO_ENTRY: the accelerator's daemon has to be one
    # selfdrived ignores, or losing it costs the drive instead of the big model
    from openpilot.sunnypilot import jetlink_adapter
    from openpilot.system.manager.process_config import managed_processes
    self.assertEqual(AcceleratorEvents.OPTIONAL_PROCESSES, {jetlink_adapter.OWNER})
    self.assertIn('jetlinkd', managed_processes)
