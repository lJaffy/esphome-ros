"""Worker-track checks (stdlib-only: no esphome install needed).

Guards the Phase-2 threading split: all uxr_* calls run on one pinned
worker task, the loop thread only enqueues memcpy payloads, and no mutex
or heap container exists on the data path. Asserts via source grep.
"""
import re
from pathlib import Path

REPO = Path(__file__).parent.parent
XCOMP = REPO / "esphome" / "components" / "xrce_dds"
RCOMP = REPO / "esphome" / "components" / "ros2"
XH = (XCOMP / "xrce_dds_component.h").read_text()
XCPP = (XCOMP / "xrce_dds_component.cpp").read_text()
RH = (RCOMP / "ros2_component.h").read_text()
RCPP = (RCOMP / "ros2_component.cpp").read_text()


def loop_body():
    m = re.search(r"void XrceDdsComponent::loop\(\) \{(.*?)\n\}", XCPP, re.DOTALL)
    assert m, "xrce loop() not found"
    return m.group(1)


def test_worker_task_topology():
    assert "xTaskCreatePinnedToCore" in XCPP
    assert "XRCE_WORKER_CORE" in XH
    assert "XRCE_WORKER_PRIO" in XH
    assert "XRCE_WORKER_STACK" in XH
    assert "worker_trampoline_" in XCPP
    assert "worker_loop_" in XCPP
    assert "vTaskDelay" in XCPP


def test_queues_created_static():
    assert XCPP.count("xQueueCreateStatic") >= 3  # out + ctrl + image mailbox
    assert "xQueueCreateStatic" in RCPP  # ros2 inbound
    assert "ROS2_INBOUND_DEPTH" in RCPP
    assert "xQueueOverwrite" in XCPP  # 1-deep image mailbox
    assert "StaticQueue_t" in XH
    assert "StaticQueue_t" in RH


def test_loop_thread_never_pumps():
    body = loop_body()
    for banned in ("pump_once", "try_connect_", "create_pending_entities_",
                   "uxr_prepare_output_stream", "uxr_run_session"):
        assert banned not in body, f"loop() still calls {banned}"


def test_no_mutex_on_data_path():
    for text, name in ((XH, "xrce h"), (XCPP, "xrce cpp"), (RH, "ros2 h"), (RCPP, "ros2 cpp")):
        assert "std::mutex" not in text, f"{name} uses std::mutex"
        assert "lock_guard" not in text, f"{name} uses lock_guard"
        assert "std::queue" not in text, f"{name} uses std::queue"
        assert "std::vector" not in text, f"{name} uses std::vector"


def test_wrappers_enqueue_without_blocking():
    assert "xQueueSend(this->out_queue_, &o, 0)" in XCPP
    assert "xQueueSend(this->ctrl_queue_, &c, 0)" in XCPP
    assert "xQueueOverwrite(this->img_box_" in XCPP
    # Drop-oldest on full outbound/inbound (freshest wins).
    assert XCPP.count("xQueueReceive(this->out_queue_, &drop, 0)") >= 1
    assert "inbound_drop_" in RH
    assert "inbound_drop_" in RCPP


def test_mailbox_overwrite_counter():
    assert "img_drop_mailbox_overwrite_" in XH
    assert "img_drop_mailbox_overwrite_" in XCPP
    assert "mailbox" in XCPP.lower()
    assert "Images ok:" in XCPP
    assert "mbox" in XCPP  # dump line covers the new reason


def test_mailbox_staged_off_stack():
    # A 48 kB item must not live on the loop task stack.
    assert "img_stage_" in XH
    assert "ImageItem &img = this->img_stage_" in XCPP


def test_atomic_link_and_counters():
    assert "std::atomic<bool> link_up_" in XH
    assert "std::atomic<uint32_t> tx_ok_" in XH
    assert "std::atomic<uint32_t> img_drop_prepare_" in XH
    assert "link_up_.load" in XCPP
    assert "#include <atomic>" in XH
    assert "#include <atomic>" in RH


def test_dump_uses_snapshots_not_tables():
    assert "snap_readers_" in XH
    assert "snap_topics_" in XH
    assert "snapshot_tables_" in XCPP
    dump = XCPP[XCPP.index("void XrceDdsComponent::dump_config"):]
    dump = dump[:dump.index("\n}\n")]
    assert "count_topics_()" not in dump, "dump reads worker-owned tables"


def test_ros2_dispatch_on_loop():
    assert "drain_inbound_" in RCPP
    assert "dispatch_sub_" in RCPP
    assert "enqueue_inbound_" in RCPP
    loop = RCPP[RCPP.index("void Ros2Component::loop"):]
    loop = loop[:loop.index("\n    }\n")]
    assert "drain_inbound_" in loop
    assert "dispatch_joints_" not in loop  # dispatches run via drain, not inline


def test_frozen_contracts_survived():
    # Middleware interface + Track-1 constants untouched by the split.
    assert "publish_image(const std::string &topic, const uint8_t *jpeg" in XH
    assert "XRCE_IMG_HISTORY" in XH
    assert "1472" in XH
    assert "IMG_PREPARE_RETRY_MS" in XCPP
    assert "IMG_ENCODE_FAIL_LIMIT" in XCPP
    assert "PROBE tx:" in XCPP
    assert "Image prepare congested" in XCPP
