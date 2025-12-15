import cv2
import json
import os
import shutil
import numpy as np
from ultralytics import YOLO
from fastapi import FastAPI, HTTPException, UploadFile, File, Form
from fastapi.responses import StreamingResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from typing import List, Dict, Any
import asyncio
from pydantic import BaseModel
import logging
import time
from datetime import datetime
import time
import re
from dotenv import load_dotenv
from report import Report, DetectionResult, ROIStatus, DetectionSummary
import torch  # <-- add
import subprocess
from pyModbusTCP.client import ModbusClient
import threading
from contextlib import asynccontextmanager

load_dotenv()

# PLC / Modbus configuration

PLC_IP = os.getenv("PLC_IP", os.getenv("IP", "192.168.250.1"))
PLC_PORT = int(os.getenv("PLC_PORT", os.getenv("PORT", "502")))
UNIT_ID = int(os.getenv("PLC_UNIT_ID", "1"))
PLC_MONITOR_ENABLED = os.getenv(
    "PLC_MONITOR_ENABLED", "1") not in ("0", "false", "False")


COIL_SENSOR_1_FOTO = int(os.getenv("COIL_SENSOR_1_FOTO", "480"))  # w30.00
# Sensor 2 (Input): Memicu sinyal reject
COIL_SENSOR_2_REJECT = int(os.getenv("COIL_SENSOR_2_REJECT", "481"))  # w30.01
# Reject (Output): Sinyal ke PLC untuk reject item
COIL_REJECT_OUTPUT = int(os.getenv("COIL_REJECT_OUTPUT", "80"))  # w5.00
# Heartbeat (Output): Sinyal "Keep-alive" (Script sedang berjalan)
COIL_HEARTBEAT = int(os.getenv("COIL_HEARTBEAT", "81"))

# ==== KONFIGURASI ====
MODEL_PATH = "model/model_11_12_2025.pt"
JSON_PATH = "roi_coordinate/roi_coords.json"
ROI_JSON_FOLDER = "roi_coordinate"
CONFIG_FOLDER = "configuration"  # Folder untuk semua file konfigurasi
CROPPED_FOLDER = "debug_rois_cropped"  # Folder untuk ROI yang di-crop
DETECTION_FOLDER = "debug_rois_detected"
OUTPUT_PATH = "detection_result.jpg"
TEMP_IMAGE_PATH = "temp_capture.jpg"
RESULTS_FOLDER = "detection_results"
NUM_ROI_FOLDER = "num_roi"
NUM_ROI_PATH = os.path.join(NUM_ROI_FOLDER, "expected_counts.json")
# NOT_GOOD_FOLDER = "nod_good"  # <-- old single folder (dipakai sebagai base)
NOT_GOOD_BASE_FOLDER = "nod_good"  # folder induk

# Path untuk menyimpan setelan kamera (pindah ke CONFIG_FOLDER)
CAMERA_SETTINGS_PATH = os.path.join(CONFIG_FOLDER, "camera_settings.json")
# Path untuk menyimpan counter PLC (pindah ke CONFIG_FOLDER)
PLC_COUNTER_PATH = os.path.join(CONFIG_FOLDER, "plc_live_counters.json")

# Add this: configurable camera source (defaults to correct Linux path)
CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "/dev/video0")

print(f"Cuda available: {torch.cuda.is_available()}")

# Preferred device for YOLO (auto)
PREFERRED_DEVICE = os.getenv(
    "YOLO_DEVICE", "cuda" if torch.cuda.is_available() else "cpu")

# ==== EXPECTED OBJECT COUNTS PER ROI ====
# ROI 1-6: 5,4,5,4,5,4
# ROI 7-12: 4,5,4,5,4,5
EXPECTED_COUNTS = {
    1: 5,  2: 4,  3: 5,  4: 4,  5: 5,  6: 4,
    7: 4,  8: 5,  9: 4,  10: 5, 11: 4, 12: 5
}

# ==== MODEL DATA ====


class ROI(BaseModel):
    x1: int
    y1: int
    x2: int
    y2: int


class ROICoordinates(BaseModel):
    coordinates: List[ROI]


# ==== INISIALISASI FASTAPI ====


@asynccontextmanager
async def app_lifespan(app: FastAPI):
    load_plc_counters()
    load_detection_config()
    load_roi_good_threshold()

    # run_v4l2_cmd([f'--set-fmt-video=width=3840,height=2160,pixelformat=MJPG'])

    resolusi_4k, out_4k = run_v4l2_cmd(
        [f'--set-fmt-video=width=3840,height=2160,pixelformat=MJPG'])
    if resolusi_4k:
        logger.info("4K resolusi")
    else:
        logger.warning(f"⚠️ Failed to disable 4k on startup: {out_4k}")

    brightness_brio, out_bright = run_v4l2_cmd(['-c', 'brightness=20'])
    if brightness_brio:
        run_v4l2_cmd(['-c', '--set-ctrl=brightness=50'])
        logger.info(f"Brightness 50")
    else:
        logger.warning(f"⚠️ Failed to disable bright on startup: {out_bright}")

    # run_v4l2_cmd([f'v4l2-ctl -d /dev/video0 --set-ctrl=brightness=50'])

    ok, out = run_v4l2_cmd(['-c', 'focus_automatic_continuous=0'])
    if ok:
        logger.info("✅ Auto focus disabled on startup")
    else:
        logger.warning(f"⚠️ Failed to disable auto focus on startup: {out}")

    start_camera_if_needed()

    apply_saved_camera_settings()
    load_plc_counters()

    try:
        await start_plc_monitor()
    except Exception as e:
        logger.warning(f"⚠️ Gagal start PLC monitor: {e}")

    initialize_models()

    try:
        yield
    finally:
        global camera_active, cap
        stop_plc_monitor()
        if cap:
            cap.release()
            camera_active = False
        logger.info("API shutdown complete")


app = FastAPI(title="YUASA", version="1.0.0", lifespan=app_lifespan)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ==== VARIABEL GLOBAL ====
model = None
areas = []
current_results = {}
current_status = {}
frame_buffer = None
camera_active = False
cap = None
original_frame = None  # Tambahkan variabel original_frame
capture_task = None
stream_clients = 0  # jumlah endpoint streaming aktif

# Tambahan: status PLC monitor
plc_status_lock = threading.Lock()
plc_monitor_thread = None
plc_monitor_running = False
plc_connected = False
plc_last_ok_ts = 0.0
plc_stop_event = threading.Event()

# ==== STATE UNTUK LOGIKA REJECT BERDASARKAN COUNTER 481 ====
part_status_db = {}           # contoh: {1: "PENDING", 2: "GOOD", 3: "NOT GOOD"}
counter_sensor_1 = 0          # Batch ID yang dipicu S1 (Foto)
counter_sensor_2 = 0          # Batch ID yang sedang diproses S2 (Reject)
plc_db_lock = threading.Lock()

# ==== SETUP LOGGING ====
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ==== FUNGSI UTILITAS ====


def ensure_folder_exists(folder_path):
    """Memastikan folder exists, jika tidak dibuat"""
    if not os.path.exists(folder_path):
        os.makedirs(folder_path, exist_ok=True)
        logger.info(f"📁 Folder created: {folder_path}")


def clean_folder(folder_path):
    """Membersihkan folder dengan menghapus semua file di dalamnya"""
    if os.path.exists(folder_path):
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                logger.error(f'Gagal menghapus {file_path}. Reason: {e}')
        logger.info(f"🗑️ Folder {folder_path} dibersihkan")
    else:
        os.makedirs(folder_path, exist_ok=True)
        logger.info(f"📁 Folder {folder_path} dibuat")


def clean_json_files(folder_path):
    """Menghapus semua file JSON dalam folder"""
    if os.path.exists(folder_path):
        json_files = [f for f in os.listdir(
            folder_path) if f.endswith('.json')]
        for json_file in json_files:
            file_path = os.path.join(folder_path, json_file)

            # Pastikan kita tidak menghapus DUA file counter
            persistent_files = []
            if 'BATTERY_COUNTER_PATH' in globals():
                persistent_files.append(os.path.abspath(BATTERY_COUNTER_PATH))
            if 'STATUS_COUNTER_PATH' in globals():
                persistent_files.append(os.path.abspath(STATUS_COUNTER_PATH))
            if 'CAMERA_SETTINGS_PATH' in globals():  # <-- Tambahan
                persistent_files.append(os.path.abspath(CAMERA_SETTINGS_PATH))

            try:
                if os.path.abspath(file_path) in persistent_files:
                    logger.info(f"ℹ️ Melewatkan file counter: {file_path}")
                    continue
            except Exception:
                pass  # jika BATTERY_COUNTER_PATH belum ada

            try:
                os.remove(file_path)
                logger.info(f"🗑️ File JSON dihapus: {file_path}")
            except Exception as e:
                logger.error(f'Gagal menghapus {file_path}. Reason: {e}')
        logger.info(
            f"🗑️ Semua file JSON di {folder_path} dihapus (kecuali counter)")


def load_roi_coordinates():
    global areas
    if not os.path.exists(JSON_PATH):
        logger.warning(f"File {JSON_PATH} tidak ditemukan!")
        areas = []
        return areas

    with open(JSON_PATH, 'r') as f:
        areas = json.load(f)

    logger.info(f"📊 Load {len(areas)} ROI dari {JSON_PATH}")
    return areas


def save_roi_coordinates(roi_list):
    with open(JSON_PATH, 'w') as f:
        json.dump(roi_list, f, indent=4)
    logger.info(f"✅ ROI coordinates saved to {JSON_PATH}")


def load_camera_settings() -> dict:
    """Membaca file camera_settings.json jika ada."""
    if not os.path.exists(CAMERA_SETTINGS_PATH):
        return {}  # Kembalikan dict kosong jika tidak ada file
    try:
        with open(CAMERA_SETTINGS_PATH, 'r') as f:
            settings = json.load(f)
            logger.info(
                f"⚙️ Berhasil memuat setelan kamera dari {CAMERA_SETTINGS_PATH}")
            return settings
    except Exception as e:
        logger.error(f"❌ Gagal membaca {CAMERA_SETTINGS_PATH}: {e}")
        return {}


def save_camera_settings(new_settings: dict):
    """Menyimpan (merge) setelan baru ke camera_settings.json."""
    ensure_folder_exists(
        CONFIG_FOLDER)  # Pastikan folder 'configuration' ada
    current_settings = load_camera_settings()  # Baca setelan lama
    current_settings.update(new_settings)  # Timpa/tambahkan dengan setelan new
    try:
        with open(CAMERA_SETTINGS_PATH, 'w') as f:
            json.dump(current_settings, f, indent=4)
        logger.info(f"💾 Setelan kamera disimpan: {new_settings}")
    except Exception as e:
        logger.error(f"❌ Gagal menyimpan setelan kamera: {e}")


def apply_saved_camera_settings():
    """Menerapkan setelan dari file JSON ke kamera v4l2."""
    logger.info("Menerapkan setelan kamera yang tersimpan dari JSON...")
    settings = load_camera_settings()
    if not settings:
        logger.info("ℹ️ Tidak ada file setelan kamera. Menggunakan default.")
        return

    # Kunci di JSON harus sama dengan perintah v4l2
    # contoh: {"zoom_absolute": 120, "focus_absolute": 10, "focus_automatic_continuous": 0}
    for key, value in settings.items():
        try:
            val = int(value)  # Pastikan nilai adalah integer
            logger.info(f"🔩 Menerapkan setelan: {key}={val}")
            ok, out = run_v4l2_cmd(['-c', f'{key}={val}'])
            if not ok:
                logger.warning(
                    f"⚠️ Gagal menerapkan setelan {key}={val}: {out}")
            else:
                logger.info(f"✅ Setelan {key}={val} berhasil diterapkan.")
        except Exception as e:
            logger.error(f"❌ Error saat menerapkan setelan {key}: {e}")


def load_plc_counters():
    """Membaca counter PLC terakhir dari JSON saat startup."""
    global counter_sensor_1, counter_sensor_2
    if not os.path.exists(PLC_COUNTER_PATH):
        logger.info("ℹ️ File counter PLC tidak ditemukan. Mulai dari 0.")
        counter_sensor_1 = 0
        counter_sensor_2 = 0
        return
    try:
        with open(PLC_COUNTER_PATH, 'r') as f:
            counters = json.load(f)
        counter_sensor_1 = int(counters.get("counter_sensor_1", 0))
        counter_sensor_2 = int(counters.get("counter_sensor_2", 0))
        logger.info(
            f"✅ Counter PLC dimuat: S1={counter_sensor_1}, S2={counter_sensor_2}")
    except Exception as e:
        logger.error(f"❌ Gagal memuat counter PLC. Reset ke 0. Error: {e}")
        counter_sensor_1 = 0
        counter_sensor_2 = 0


def save_plc_counters(s1_val, s2_val):
    """Menyimpan nilai counter PLC saat ini ke JSON."""
    try:
        tmp_path = PLC_COUNTER_PATH + ".tmp"
        with open(tmp_path, 'w') as f:
            json.dump({"counter_sensor_1": s1_val, "counter_sensor_2": s2_val,
                      "last_updated": datetime.now().isoformat()}, f, indent=4)
        os.replace(tmp_path, PLC_COUNTER_PATH)
        logger.info(f"💾 Counter PLC disimpan: S1={s1_val}, S2={s2_val}")
    except Exception as e:
        logger.error(f"❌ Gagal menyimpan counter PLC: {e}")


def get_expected_count(roi_id: int) -> int:
    """
    Return expected count for roi_id by reading num_roi/expected_counts.json (if exists).
    Fall back to in-memory EXPECTED_COUNTS when file not present or error.
    """
    try:
        if os.path.exists(NUM_ROI_PATH):
            with open(NUM_ROI_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
            for k, v in data.items():
                m = re.search(r"(\d+)", str(k))
                if m and int(m.group(1)) == roi_id:
                    if isinstance(v, int) and v >= 0:
                        return v
        # fallback
        return EXPECTED_COUNTS.get(roi_id, 0)
    except Exception as e:
        logger.warning(f"⚠️ Gagal membaca expected counts dari file: {e}")
        return EXPECTED_COUNTS.get(roi_id, 0)


def get_roi_status(roi_id, detected_count, detected_classes=None):
    """Menentukan status ROI berdasarkan jumlah objek yang terdeteksi.
    Membaca nilai expected dari file num_roi/expected_counts.json jika tersedia.
    """
    expected_count = get_expected_count(roi_id)

    if detected_count >= expected_count:
        return "GOOD", f"Expected: {expected_count}, Detected: {detected_count}"
    elif detected_count < expected_count:
        return "NOT GOOD", f"Missing {expected_count - detected_count} objects (Expected: {expected_count}, Detected: {detected_count})"
    else:
        return "NOT GOOD", f"Extra {detected_count - expected_count} objects (Expected: {expected_count}, Detected: {detected_count})"


def save_detection_results(results_data, timestamp):
    """Menyimpan hasil deteksi ke file JSON dalam folder results"""
    ensure_folder_exists(RESULTS_FOLDER)

    # Format filename dengan timestamp
    filename = f"detection_{timestamp.strftime('%Y%m%d_%H%M%S')}.json"
    filepath = os.path.join(RESULTS_FOLDER, filename)

    # Tambahkan metadata
    results_data['metadata'] = {
        'timestamp': timestamp.isoformat(),
        'filename': filename,
    }

    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(results_data, f, indent=4, ensure_ascii=False)

    logger.info(f"💾 Hasil deteksi disimpan: {filepath}")
    return filepath


# ----- Battery counter (persistent JSON) ---------------------------------
BATTERY_COUNTER_PATH = os.path.join(RESULTS_FOLDER, "battery_counter.json")

# File untuk menghitung total GOOD vs NOT GOOD
STATUS_COUNTER_PATH = os.path.join(RESULTS_FOLDER, "status_counter.json")


def get_next_battery_id_json():
    """Read/increment persistent battery counter stored as JSON in RESULTS_FOLDER."""
    ensure_folder_exists(RESULTS_FOLDER)
    try:
        if os.path.exists(BATTERY_COUNTER_PATH):
            with open(BATTERY_COUNTER_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            v = int(data.get("counter", 0))
        else:
            v = 0
        v += 1
        print(f"🔢 Next battery_id: {v}")
        tmp = BATTERY_COUNTER_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"counter": v}, f)
        os.replace(tmp, BATTERY_COUNTER_PATH)
        logger.info(f"🔢 Next battery_id: {v}")
        return v
    except Exception as e:
        logger.error(f"❌ Gagal update battery counter: {e}")
        return 1


def update_status_counter(status: str):
    """Membaca, menambah, dan menyimpan counter status GOOD/NOT_GOOD."""
    ensure_folder_exists(RESULTS_FOLDER)
    data = {"good": 0, "not_good": 0, "total_processed": 0}

    try:
        if os.path.exists(STATUS_COUNTER_PATH):
            with open(STATUS_COUNTER_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                if "good" not in data:
                    data["good"] = 0
                if "not_good" not in data:
                    data["not_good"] = 0
    except Exception as e:
        logger.warning(
            f"⚠️ Gagal membaca status counter, membuat file baru. Error: {e}")
        data = {"good": 0, "not_good": 0, "total_processed": 0}

    if status == "GOOD":
        data["good"] += 1
    else:
        data["not_good"] += 1

    data["total_processed"] = data["good"] + data["not_good"]
    data["last_updated"] = datetime.now().isoformat()

    try:
        tmp_path = STATUS_COUNTER_PATH + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)
        os.replace(tmp_path, STATUS_COUNTER_PATH)
        logger.info(
            f"📊 Status counter diperbarui: GOOD={data['good']}, NOT_GOOD={data['not_good']}")
    except Exception as e:
        logger.error(f"❌ Gagal menyimpan status counter: {e}")


def safe_remove_folder(path: str):
    try:
        if os.path.isdir(path):
            shutil.rmtree(path)
            logger.info(f"🗑️ Folder dihapus (post-reporting): {path}")
    except Exception as e:
        logger.error(f"❌ Gagal hapus folder {path}: {e}")


async def report_results_async(overall_status, NOT_GOOD_FOLDER, output_path, battery_id):
    """Background task: send reporting to external service and save summary to disk (pakai class)."""
    reporting_summary = {"battery": None, "cells": []}
    try:
        grpc_address = os.getenv("GRPC_ADDRESS", "http://10.42.0.1:8080")
        report = await Report.create(grpc_address)

        # Optional start
        try:
            await report.start_processing()
        except Exception as e_start:
            logger.warning(
                f"⚠️ start_processing failed or not supported: {e_start}")

        status_for_report = "good" if overall_status == "GOOD" else "not_good"

        # Build DetectionResult object
        try:
            global current_results
            if current_results:
                roi_counts = {str(k): int(v) for k, v in current_results.get(
                    "roi_counts", {}).items()}
                raw_statuses = current_results.get("roi_statuses", {})
                roi_statuses = {
                    str(k): ROIStatus(
                        status=v.get("status", ""),
                        message=v.get("message", ""),
                        expected=int(v.get("expected", 0)),
                        detected=int(v.get("detected", 0))
                    )
                    for k, v in raw_statuses.items()
                }
                good_rois = sum(1 for v in roi_statuses.values()
                                if v.status == "GOOD")
                total_rois = len(roi_statuses)
                detection_summary_obj = DetectionSummary(
                    good_rois=good_rois,
                    not_good_rois=total_rois - good_rois,
                    total_rois=total_rois,
                    total_detected_objects=int(
                        current_results.get("total_objects", 0))
                )
                detection_result_obj = DetectionResult(
                    total_objects=int(current_results.get("total_objects", 0)),
                    roi_counts=roi_counts,
                    roi_statuses=roi_statuses,
                    overall_status=current_results.get(
                        "overall_status", overall_status),
                    result_image=current_results.get(
                        "image_path", output_path),
                    timestamp=datetime.now().isoformat(),
                    roi_total=total_rois,
                    detection_summary=detection_summary_obj
                )
            else:
                # Fallback kosong
                roi_statuses = {}
                for i in range(1, 13):
                    exp = EXPECTED_COUNTS.get(i, 0)
                    roi_statuses[str(i)] = ROIStatus(
                        status="NOT GOOD",
                        message=f"Missing {exp} objects (Expected: {exp}, Detected: 0)",
                        expected=exp,
                        detected=0
                    )
                detection_summary_obj = DetectionSummary(
                    good_rois=0,
                    not_good_rois=len(roi_statuses),
                    total_rois=len(roi_statuses),
                    total_detected_objects=0
                )
                detection_result_obj = DetectionResult(
                    total_objects=0,
                    roi_counts={k: 0 for k in roi_statuses.keys()},
                    roi_statuses=roi_statuses,
                    overall_status="NOT GOOD",
                    result_image=output_path,
                    timestamp=datetime.now().isoformat(),
                    roi_total=len(roi_statuses),
                    detection_summary=detection_summary_obj
                )
        except Exception as e_prep:
            logger.error(
                f"⚠️ Gagal membangun DetectionResult object: {e_prep}")
            return  # tidak lanjut laporan

        # Kirim battery
        try:
            resp_batt = await report.report_battery(
                battery_id=battery_id,
                image_path=output_path,
                status=status_for_report,
                detection_result=detection_result_obj
            )
        except Exception as e_rep:
            logger.error(f"❌ report_battery error: {e_rep}")
            reporting_summary["error"] = f"report_battery_failed: {e_rep}"
            resp_batt = None

        reporting_summary["battery"] = {
            "status_code": getattr(getattr(resp_batt, "base_response", None), "status_code", None),
            "report_battery_id": getattr(resp_batt, "report_battery_id", None)
        }

        # Laporkan setiap ROI NOT GOOD (pakai folder)
        if resp_batt and getattr(getattr(resp_batt, "base_response", None), "status_code", None) == 200:
            report_battery_id = getattr(resp_batt, "report_battery_id", None)
            if report_battery_id:
                roi_files = []
                if os.path.isdir(NOT_GOOD_FOLDER):
                    roi_files = [
                        os.path.join(NOT_GOOD_FOLDER, f)
                        for f in os.listdir(NOT_GOOD_FOLDER)
                        if f.startswith("roi_")
                    ]
                    roi_files.sort()
                for fpath in roi_files:
                    print(fpath)
                    m = re.search(r"roi_(\d+)_", os.path.basename(fpath))
                    cell_id = int(m.group(1)) if m else 1
                    try:
                        resp_cell = await report.report_cell(
                            report_battery_id=report_battery_id,
                            cell_id=cell_id,
                            image_path=fpath,
                            status=status_for_report
                        )
                        reporting_summary["cells"].append({
                            "file": fpath,
                            "cell_id": cell_id,
                            "status_code": getattr(getattr(resp_cell, "base_response", None), "status_code", None)
                        })
                    except Exception as e_cell:
                        logger.error(f"Error reporting cell {fpath}: {e_cell}")
                        reporting_summary["cells"].append(
                            {"file": fpath, "error": str(e_cell)})
        else:
            reporting_summary.setdefault("error", "report_battery_failed")

    except Exception as e:
        logger.error(f"Error reporting results: {e}")
        reporting_summary["error"] = str(e)

    # Simpan ringkasan
    try:
        rpt_path = os.path.join(
            RESULTS_FOLDER, f"reporting_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
        with open(rpt_path, "w", encoding="utf-8") as f:
            json.dump(reporting_summary, f, indent=2, ensure_ascii=False)
        logger.info(f"📄 Reporting summary saved: {rpt_path}")

    except Exception as e:
        logger.error(f"❌ Gagal menyimpan reporting summary: {e}")

    # Hapus folder NOT_GOOD_FOLDER setelah semua file berhasil dikirim
    # Cek apakah semua cell berhasil dikirim (tidak ada error)
    all_cells_sent = True
    if reporting_summary.get("cells"):
        for cell in reporting_summary["cells"]:
            if "error" in cell or cell.get("status_code") != 200:
                all_cells_sent = False
                break
    
    # Hanya hapus folder jika battery report sukses DAN semua cell terkirim
    battery_success = reporting_summary.get("battery", {}).get("status_code") == 200
    no_general_error = "error" not in reporting_summary
    
    if battery_success and all_cells_sent and no_general_error:
        safe_remove_folder(NOT_GOOD_FOLDER)
        logger.info(f"✅ Folder {NOT_GOOD_FOLDER} berhasil dihapus setelah semua file terkirim")
    else:
        logger.warning(f"⚠️ Folder {NOT_GOOD_FOLDER} TIDAK dihapus karena ada file yang gagal dikirim. Akan dicoba lagi nanti atau dihapus manual.")

RECONNECT_SLEEP_SEC = float(os.getenv("CAMERA_RECONNECT_SLEEP", "1.0"))
RECONNECT_MAX_ATTEMPTS = int(os.getenv("CAMERA_RECONNECT_MAX_ATTEMPTS", "30"))


def _open_camera():
    """Membuka kamera sekali dan set properti dasar. Return True jika sukses."""
    global cap
    try:
        if cap:
            try:
                cap.release()
            except Exception:
                pass
        cap = cv2.VideoCapture(CAMERA_SOURCE)
        time.sleep(0.3)
        if not cap.isOpened():
            return False
        # Set resolusi (best-effort)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        return True
    except Exception as e:
        logger.error(f"Open camera exception: {e}")
        return False


def ensure_camera_open():
    """
    Pastikan kamera terbuka; jika tidak, lakukan retry hingga RECONNECT_MAX_ATTEMPTS.
    Return True jika kamera siap, False jika gagal total.
    """
    global cap, camera_active
    if cap and getattr(cap, "isOpened", lambda: False)():
        return True
    logger.warning("Kamera tidak terbuka. Memulai proses reconnect...")
    for attempt in range(1, RECONNECT_MAX_ATTEMPTS + 1):
        ok = _open_camera()
        if ok:
            logger.info(f"Reconnect kamera berhasil pada attempt {attempt}")
            return True
        logger.warning(
            f"Attempt {attempt}/{RECONNECT_MAX_ATTEMPTS} gagal. Coba lagi dalam {RECONNECT_SLEEP_SEC}s...")
        time.sleep(RECONNECT_SLEEP_SEC)
    logger.error("Gagal reconnect kamera setelah semua percobaan.")
    camera_active = False
    return False


def start_camera_if_needed():
    global camera_active, capture_task
    if not camera_active:
        camera_active = True
        logger.info("Menyalakan kamera & memulai loop capture...")
        # Try initial open (non-fatal if gagal, loop akan coba lagi)
        ensure_camera_open()
        capture_task = asyncio.create_task(camera_capture_loop())


async def camera_capture_loop():
    global cap, camera_active, original_frame
    consecutive_fail = 0
    max_fail_before_reconnect = 5
    try:
        while camera_active:
            if not ensure_camera_open():
                logger.error("Stop loop: kamera tidak bisa direconnect.")
                break

            ret, frame = cap.read()
            if not ret or frame is None:
                consecutive_fail += 1
                logger.warning(f"Frame read gagal (#{consecutive_fail}).")
                if consecutive_fail >= max_fail_before_reconnect:
                    logger.warning(
                        "Mencapai batas gagal frame. Coba reconnect kamera...")
                    ensure_camera_open()
                    consecutive_fail = 0
                await asyncio.sleep(0.05)
                continue

            consecutive_fail = 0
            original_frame = frame
            await asyncio.sleep(0)  # cooperative
    except Exception as e:
        logger.error(f"Loop capture error: {e}")
    finally:
        logger.info("Loop capture berhenti")


def capture_image_from_camera():
    global original_frame, cap
    # Tambahan: pastikan kamera siap sebelum capture
    if not ensure_camera_open():
        raise RuntimeError("Kamera tidak tersedia untuk capture")
    # Jika ada frame dari stream aktif, gunakan itu (menghindari membuka kamera lagi)
    if original_frame is not None:
        try:
            # original_frame sudah numpy array BGR
            cv2.imwrite(TEMP_IMAGE_PATH, original_frame)
            logger.info(
                f"✅ Menggunakan frame dari buffer dan disimpan sebagai: {TEMP_IMAGE_PATH}")
            return TEMP_IMAGE_PATH
        except Exception as e:
            logger.warning(f"⚠️ Gagal menggunakan original_frame: {e}")

    # Jika tidak ada frame_buffer, gunakan global cap jika tersedia
    if cap is None or not getattr(cap, 'isOpened', lambda: False)():
        temp_cap = cv2.VideoCapture(CAMERA_SOURCE)
        opened_temp = True
        time.sleep(1)
        if not temp_cap.isOpened():
            raise ValueError(
                f"Tidak dapat mengakses kamera! source={CAMERA_SOURCE}")
        logger.info("📷 Mengambil gambar dari kamera (temp)...")

        # Baca beberapa frame untuk membersihkan buffer
        for _ in range(5):
            ret, frame = temp_cap.read()
            if not ret:
                raise ValueError("Gagal membaca frame dari kamera (temp)!")

        ret, frame = temp_cap.read()
        if not ret:
            raise ValueError("Gagal mengambil gambar dari kamera (temp)!")

        cv2.imwrite(TEMP_IMAGE_PATH, frame)
        logger.info(
            f"✅ Gambar berhasil diambil (temp) dan disimpan sebagai: {TEMP_IMAGE_PATH}")
        return TEMP_IMAGE_PATH
    else:
        # Gunakan shared global cap (mis. saat /frame sedang aktif)
        logger.info("📷 Mengambil gambar dari kamera (shared cap)...")
        # Baca beberapa frame singkat untuk memastikan frame baru
        for _ in range(3):
            ret, frame = cap.read()
            if not ret:
                logger.warning(
                    "⚠️ shared cap read returned no frame, mencoba ulang...")
                time.sleep(0.1)
        ret, frame = cap.read()
        if not ret:
            raise ValueError("Gagal mengambil gambar dari shared camera!")
        cv2.imwrite(TEMP_IMAGE_PATH, frame)
        logger.info(
            f"✅ Gambar berhasil diambil (shared) dan disimpan sebagai: {TEMP_IMAGE_PATH}")
        return TEMP_IMAGE_PATH


detection_config = {
    "confidence": 0.45,
    "area_ratio_threshold": 0.77,
    "iou_threshold": 0.7,
    # "imgsz": 960
}
detection_config_lock = threading.Lock()

# Path untuk menyimpan konfigurasi deteksi (pindah ke CONFIG_FOLDER)
DETECTION_CONFIG_PATH = os.path.join(CONFIG_FOLDER, "detection_config.json")


def load_detection_config():
    """Membaca konfigurasi deteksi dari file JSON."""
    global detection_config
    try:
        if not os.path.exists(DETECTION_CONFIG_PATH):
            logger.info(
                "ℹ️ File detection config tidak ditemukan. Menggunakan default.")
            save_detection_config(detection_config)
            return detection_config

        with open(DETECTION_CONFIG_PATH, 'r') as f:
            loaded_config = json.load(f)

        with detection_config_lock:
            detection_config.update(loaded_config)

        logger.info(f"✅ Detection config dimuat: {detection_config}")
        return detection_config
    except Exception as e:
        logger.error(f"❌ Gagal memuat detection config: {e}")
        return detection_config


def save_detection_config(config: dict):
    """Menyimpan konfigurasi deteksi ke file JSON."""
    try:
        ensure_folder_exists(CONFIG_FOLDER)
        tmp_path = DETECTION_CONFIG_PATH + ".tmp"
        with open(tmp_path, 'w') as f:
            json.dump(config, f, indent=4)
        os.replace(tmp_path, DETECTION_CONFIG_PATH)
        logger.info(f"💾 Detection config disimpan: {config}")
    except Exception as e:
        logger.error(f"❌ Gagal menyimpan detection config: {e}")
        raise


def remove_nested_boxes(boxes, area_ratio_threshold=0.7, iou_threshold=0.5):
    """
    Filter nested boxes dan boxes yang berdekatan (overlapping)

    Args:
        boxes: List of [x1, y1, x2, y2, conf, cls]
        area_ratio_threshold: Threshold untuk nested boxes (default: 0.7)
        iou_threshold: Threshold IoU untuk boxes yang berdekatan (default: 0.5)
    2
    Returns:
        filtered: List of boxes setelah filtering
    """
    if len(boxes) == 0:
        return []

    # Sort boxes by confidence (descending)
    boxes_sorted = sorted(boxes, key=lambda x: x[4], reverse=True)

    filtered = []

    for i, box_a in enumerate(boxes_sorted):
        xa1, ya1, xa2, ya2, conf_a, cls_a = box_a
        area_a = (xa2 - xa1) * (ya2 - ya1)

        should_keep = True

        for box_b in filtered:
            xb1, yb1, xb2, yb2, conf_b, cls_b = box_b
            area_b = (xb2 - xb1) * (yb2 - yb1)

            # Check if box_a is nested inside box_b
            if xa1 >= xb1 and ya1 >= yb1 and xa2 <= xb2 and ya2 <= yb2:
                if area_a < area_b * area_ratio_threshold:
                    should_keep = False
                    break

            # Calculate IoU (Intersection over Union) untuk deteksi overlap
            # Intersection area
            x_left = max(xa1, xb1)
            y_top = max(ya1, yb1)
            x_right = min(xa2, xb2)
            y_bottom = min(ya2, yb2)

            if x_right > x_left and y_bottom > y_top:
                intersection_area = (x_right - x_left) * (y_bottom - y_top)
                union_area = area_a + area_b - intersection_area
                iou = intersection_area / union_area if union_area > 0 else 0

                # Jika IoU tinggi, buang box dengan confidence lebih rendah
                if iou > iou_threshold:
                    should_keep = False
                    break

        if should_keep:
            filtered.append(box_a)

    return filtered


roi_good_threshold_config = {
    "min_good_rois": 6,  # Default: minimal 6 ROI harus GOOD
    "mode": "absolute"   # "absolute" atau "percentage"
}
roi_good_threshold_lock = threading.Lock()

# Path untuk menyimpan konfigurasi threshold (pindah ke CONFIG_FOLDER)
ROI_GOOD_THRESHOLD_PATH = os.path.join(
    CONFIG_FOLDER, "roi_good_threshold.json")


def load_roi_good_threshold():
    """Membaca konfigurasi threshold ROI GOOD dari file JSON."""
    global roi_good_threshold_config
    try:
        if not os.path.exists(ROI_GOOD_THRESHOLD_PATH):
            logger.info(
                "ℹ️ File ROI good threshold tidak ditemukan. Menggunakan default.")
            save_roi_good_threshold(roi_good_threshold_config)
            return roi_good_threshold_config

        with open(ROI_GOOD_THRESHOLD_PATH, 'r') as f:
            loaded_config = json.load(f)

        with roi_good_threshold_lock:
            roi_good_threshold_config.update(loaded_config)

        logger.info(
            f"✅ ROI good threshold config dimuat: {roi_good_threshold_config}")
        return roi_good_threshold_config
    except Exception as e:
        logger.error(f"❌ Gagal memuat ROI good threshold config: {e}")
        return roi_good_threshold_config


def save_roi_good_threshold(config: dict):
    """Menyimpan konfigurasi threshold ROI GOOD ke file JSON."""
    try:
        ensure_folder_exists(CONFIG_FOLDER)
        tmp_path = ROI_GOOD_THRESHOLD_PATH + ".tmp"
        with open(tmp_path, 'w') as f:
            json.dump(config, f, indent=4)
        os.replace(tmp_path, ROI_GOOD_THRESHOLD_PATH)
        logger.info(f"💾 ROI good threshold config disimpan: {config}")
    except Exception as e:
        logger.error(f"❌ Gagal menyimpan ROI good threshold config: {e}")
        raise


def check_overall_status(good_rois: int, total_rois: int) -> str:
    """
    Menentukan overall status berdasarkan threshold yang dikonfigurasi.

    Args:
        good_rois: Jumlah ROI yang berstatus GOOD
        total_rois: Total jumlah ROI

    Returns:
        "GOOD" atau "NOT GOOD"
    """
    with roi_good_threshold_lock:
        min_good = roi_good_threshold_config["min_good_rois"]
        mode = roi_good_threshold_config.get("mode", "absolute")

    if mode == "percentage":
        # Mode persentase: min_good adalah persentase (0-100)
        required_good = int(total_rois * (min_good / 100))
        is_good = good_rois >= required_good
        logger.info(
            f"📊 Mode percentage: {good_rois}/{total_rois} ROI GOOD (required: {required_good} = {min_good}%)")
    else:
        # Mode absolute: min_good adalah jumlah mutlak
        is_good = good_rois >= min_good
        logger.info(
            f"📊 Mode absolute: {good_rois}/{total_rois} ROI GOOD (required: {min_good})")

    return "GOOD" if is_good else "NOT GOOD"


def process_image_with_roi(image_path, areas, model, not_good_folder):
    """Fungsi untuk memproses gambar dengan ROI TANPA UPSCALE (folder NOT GOOD per battery)."""
    start_time = time.time()
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Gagal memuat gambar: {image_path}")

    # Ambil konfigurasi deteksi terkini
    with detection_config_lock:
        conf_threshold = detection_config["confidence"]
        area_ratio = detection_config["area_ratio_threshold"]
        iou_thresh = detection_config["iou_threshold"]
        # img_size = detection_config["imgsz"]

    h, w = image.shape[:2]
    image_vis = image.copy()
    results_all = []
    total_objects = 0
    roi_counts = {}
    roi_statuses = {}

    # imgsz={img_size}
    logger.info(
        f"🔍 Memulai proses deteksi dengan config: conf={conf_threshold}, area_ratio={area_ratio}, iou={iou_thresh}")

    for i, area in enumerate(areas):
        roi_id = i + 1
        x1, y1, x2, y2 = map(
            int, [area["x1"], area["y1"], area["x2"], area["y2"]])

        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h))

        if x2 <= x1 or y2 <= y1:
            logger.warning(
                f"⚠️ ROI #{roi_id} invalid after clamping: ({x1},{y1})-({x2},{y2}), dilewati.")
            roi_counts[roi_id] = 0
            status, message = get_roi_status(roi_id, 0)
            roi_statuses[roi_id] = {
                "status": status,
                "message": message,
                "expected": EXPECTED_COUNTS.get(roi_id, 0),
                "detected": 0
            }
            continue

        roi_crop = image[y1:y2, x1:x2].copy()

        if roi_crop is None or roi_crop.size == 0 or roi_crop.shape[0] == 0 or roi_crop.shape[1] == 0:
            logger.warning(f"⚠️ ROI #{roi_id} kosong setelah crop, dilewati.")
            roi_counts[roi_id] = 0
            status, message = get_roi_status(roi_id, 0)
            roi_statuses[roi_id] = {
                "status": status,
                "message": message,
                "expected": EXPECTED_COUNTS.get(roi_id, 0),
                "detected": 0
            }
            continue

        cropped_path = os.path.join(
            CROPPED_FOLDER, f"roi_{roi_id}_cropped.jpg")
        cv2.imwrite(cropped_path, roi_crop)
        logger.info(
            f"📸 ROI #{roi_id} berhasil di-crop dan disimpan: {cropped_path}")

        try:
            logger.info(f"🔄 Memproses ROI #{roi_id}...")
            if roi_crop.ndim == 2:
                roi_crop = cv2.cvtColor(roi_crop, cv2.COLOR_GRAY2BGR)
            if roi_crop.dtype != np.uint8:
                roi_crop = roi_crop.astype(np.uint8)

            try:
                roi_results = model.predict(
                    source=roi_crop, conf=conf_threshold, iou=iou_thresh, verbose=False, device=PREFERRED_DEVICE, imgsz=960
                )
            except Exception as e_infer:
                logger.warning(
                    f"⚠️ Inference gagal di device={PREFERRED_DEVICE}: {e_infer}. Retry pakai CPU.")
                roi_results = model.predict(
                    # imgsz=img_size
                    source=roi_crop, conf=conf_threshold, verbose=False, iou=iou_thresh, device="cpu"
                )

            roi_boxes = roi_results[0].boxes
            box_list = []
            detected_classes = []
            for b in roi_boxes:
                bx1, by1, bx2, by2 = map(int, b.xyxy[0])
                conf = float(b.conf[0])
                cls = int(b.cls[0])
                class_name = model.names[cls]
                box_list.append([bx1, by1, bx2, by2, conf, cls])
                detected_classes.append(class_name)

            box_list = remove_nested_boxes(box_list, area_ratio, iou_thresh)

            count = len(box_list)
            total_objects += count
            roi_counts[roi_id] = count

            status, message = get_roi_status(roi_id, count, detected_classes)
            roi_statuses[roi_id] = {
                "status": status,
                "message": message,
                "expected": EXPECTED_COUNTS.get(roi_id, 0),
                "detected": count,
                "detected_classes": detected_classes
            }

            logger.info(
                f"📦 ROI #{roi_id}: {count} objek terdeteksi - {status}")

            if status != "GOOD":
                try:
                    ng_filename = f"roi_{roi_id}_not_good_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                    ng_path = os.path.join(not_good_folder, ng_filename)
                    cv2.imwrite(ng_path, roi_crop)
                    logger.info(f"📁 ROI #{roi_id} disimpan: {ng_path}")
                except Exception as e_ng:
                    logger.error(f"❌ Gagal simpan ROI #{roi_id}: {e_ng}")

            result_vis_roi = roi_crop.copy()
            for bx1, by1, bx2, by2, conf, cls in box_list:
                label = f"{model.names[cls]} {conf:.2f}"
                cv2.rectangle(result_vis_roi, (bx1, by1),
                              (bx2, by2), (0, 255, 0), 2)

                orig_x1 = x1 + bx1
                orig_y1 = y1 + by1
                orig_x2 = x1 + bx2
                orig_y2 = y1 + by2

                results_all.append({
                    "roi_id": roi_id,
                    "label": model.names[cls],
                    "confidence": round(conf, 3),
                    "bbox_original": [orig_x1, orig_y1, orig_x2, orig_y2],
                    "bbox_roi": [bx1, by1, bx2, by2],
                    "cropped_image_path": cropped_path
                })
                cv2.rectangle(image_vis, (orig_x1, orig_y1),
                              (orig_x2, orig_y2), (0, 255, 0), 2)

            detection_path = os.path.join(
                DETECTION_FOLDER, f"roi_{roi_id}_detected.jpg")
            cv2.imwrite(detection_path, result_vis_roi)

            color = (0, 255, 0) if status == "GOOD" else (0, 0, 255)
            cv2.rectangle(image_vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(image_vis, f"ROI {roi_id} ({count})", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        except Exception as e:
            logger.error(f"⚠️ Error processing ROI {roi_id}: {e}")
            roi_counts[roi_id] = 0
            status, message = get_roi_status(roi_id, 0)
            roi_statuses[roi_id] = {
                "status": status,
                "message": message,
                "expected": EXPECTED_COUNTS.get(roi_id, 0),
                "detected": 0
            }
            cv2.rectangle(image_vis, (x1, y1), (x2, y2),
                          (0, 0, 255), 2)  # Merah untuk error

    # Tambahkan informasi total
    cv2.putText(image_vis, f"Total Objek: {total_objects}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(image_vis, f"Jumlah ROI: {len(areas)}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    # Hitung status overall
    good_rois = sum(1 for status in roi_statuses.values()
                    if status["status"] == "GOOD")
    total_rois = len(roi_statuses)
    overall_status = check_overall_status(good_rois, total_rois)

    # Tampilkan status overall
    color = (0, 255, 0) if overall_status == "GOOD" else (0, 0, 255)
    cv2.putText(image_vis, f"STATUS: {overall_status} ({good_rois}/{total_rois})", (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)

    # Simpan hasil gambar
    cv2.imwrite(OUTPUT_PATH, image_vis)
    end_time = time.time()
    print(f"✅ Proses deteksi selesai dalam {end_time - start_time:.2f} detik")

    return image_vis, results_all, total_objects, roi_counts, roi_statuses, overall_status


@app.post("/roi_threshold")
async def set_roi_good_threshold(payload: dict):
    """
    Update threshold minimum ROI GOOD untuk overall status GOOD.

    Payload example (mode absolute):
    {
        "min_good_rois": 6,
        "mode": "absolute"
    }

    Payload example (mode percentage):
    {
        "min_good_rois": 50,
        "mode": "percentage"
    }
    """
    global roi_good_threshold_config
    try:
        updated = {}

        if "min_good_rois" in payload:
            min_good = payload["min_good_rois"]
            mode = payload.get(
                "mode", roi_good_threshold_config.get("mode", "absolute"))

            if mode == "percentage":
                # Validasi untuk mode percentage (0-100)
                if not isinstance(min_good, (int, float)) or not 0 <= min_good <= 100:
                    raise HTTPException(
                        status_code=400,
                        detail="min_good_rois untuk mode percentage harus antara 0-100"
                    )
                updated["min_good_rois"] = float(min_good)
            else:
                # Validasi untuk mode absolute
                if not isinstance(min_good, int) or min_good < 0:
                    raise HTTPException(
                        status_code=400,
                        detail="min_good_rois untuk mode absolute harus integer >= 0"
                    )
                updated["min_good_rois"] = int(min_good)

        if "mode" in payload:
            mode = payload["mode"]
            if mode not in ["absolute", "percentage"]:
                raise HTTPException(
                    status_code=400,
                    detail="mode harus 'absolute' atau 'percentage'"
                )
            updated["mode"] = mode

        if not updated:
            raise HTTPException(
                status_code=400,
                detail="Tidak ada parameter valid yang diberikan"
            )

        with roi_good_threshold_lock:
            roi_good_threshold_config.update(updated)
            save_roi_good_threshold(roi_good_threshold_config)

        logger.info(f"✅ ROI good threshold config diperbarui: {updated}")

        return {
            "status": "success",
            "message": "ROI good threshold config berhasil diperbarui",
            "config": roi_good_threshold_config
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error updating ROI good threshold config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/roi_threshold")
async def get_roi_good_threshold():
    """Mendapatkan konfigurasi threshold ROI GOOD saat ini."""
    with roi_good_threshold_lock:
        config = roi_good_threshold_config.copy()
    return {
        "status": "success",
        "config": config
    }


@app.post("/detection_config")
async def set_detection_config(payload: dict):
    """
    Update konfigurasi deteksi (confidence, area_ratio_threshold, iou_threshold, imgsz).

    Payload example:
    {
        "confidence": 0.5,
        "area_ratio_threshold": 0.75,
        "iou_threshold": 0.6,
    }
    """
    global detection_config
    try:
        updated = {}

        if "confidence" in payload:
            conf = float(payload["confidence"])
            if not 0.0 <= conf <= 1.0:
                raise HTTPException(
                    status_code=400, detail="Confidence harus antara 0.0 dan 1.0")
            updated["confidence"] = conf

        if "area_ratio_threshold" in payload:
            area_ratio = float(payload["area_ratio_threshold"])
            if not 0.0 <= area_ratio <= 1.0:
                raise HTTPException(
                    status_code=400, detail="Area ratio threshold harus antara 0.0 dan 1.0")
            updated["area_ratio_threshold"] = area_ratio

        if "iou_threshold" in payload:
            iou = float(payload["iou_threshold"])
            if not 0.0 <= iou <= 1.0:
                raise HTTPException(
                    status_code=400, detail="IoU threshold harus antara 0.0 dan 1.0")
            updated["iou_threshold"] = iou

        # if "imgsz" in payload:
        #     imgsz = int(payload["imgsz"])
        #     if imgsz <= 0:
        #         raise HTTPException(status_code=400, detail="Image size harus > 0")
        #     updated["imgsz"] = imgsz

        if not updated:
            raise HTTPException(
                status_code=400, detail="Tidak ada parameter valid yang diberikan")

        with detection_config_lock:
            detection_config.update(updated)
            save_detection_config(detection_config)

        logger.info(f"✅ Detection config diperbarui: {updated}")

        return {
            "status": "success",
            "message": "Detection config berhasil diperbarui",
            "config": detection_config
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Error updating detection config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/detection_config")
async def get_detection_config():
    """Mendapatkan konfigurasi deteksi saat ini."""
    with detection_config_lock:
        config = detection_config.copy()
    return {
        "status": "success",
        "config": config
    }


def initialize_models():
    global model
    try:
        # Load YOLO model
        model = YOLO(MODEL_PATH)
        # move to preferred device (with safe fallback)
        try:
            model.to(PREFERRED_DEVICE)
            logger.info(
                f"✅ Model YOLO berhasil dimuat (device={PREFERRED_DEVICE})")
        except Exception as e_dev:
            logger.warning(
                f"⚠️ Gagal set device {PREFERRED_DEVICE}: {e_dev}. Fallback ke CPU.")
            model.to("cpu")

        # Model Super Resolution DIHAPUS karena upscale dinonaktifkan
        logger.info("ℹ️ Fitur upscale dinonaktifkan")

        # Load ROI coordinates
        load_roi_coordinates()

        # Load expected counts from file if present
        load_expected_counts()

        # Pastikan folder results exists
        ensure_folder_exists(RESULTS_FOLDER)
        ensure_folder_exists(CROPPED_FOLDER)  # Pastikan folder cropped exists
        ensure_folder_exists(NUM_ROI_FOLDER)  # Pastikan folder num_roi exists
        ensure_folder_exists(CONFIG_FOLDER)   # Pastikan folder configuration exists
        # <-- ensure not_good folder exists
        ensure_folder_exists(NOT_GOOD_BASE_FOLDER)  # hanya base

        logger.info("📋 Expected counts per ROI:")
        for roi_id, expected in EXPECTED_COUNTS.items():
            logger.info(f"  ROI {roi_id}: {expected} objects")

        return True
    except Exception as e:
        logger.error(f"❌ Gagal memuat model: {e}")
        return False


def load_expected_counts():
    """Load EXPECTED_COUNTS from NUM_ROI_PATH if exists. Returns the dict (or keeps defaults)."""
    global EXPECTED_COUNTS
    try:
        if not os.path.exists(NUM_ROI_PATH):
            logger.info(
                f"ℹ️ Expected counts file not found at {NUM_ROI_PATH}, using defaults")
            return EXPECTED_COUNTS
        with open(NUM_ROI_PATH, 'r', encoding='utf-8') as f:
            data = json.load(f)
        # Normalize keys to int and validate values
        new_counts = {}
        for k, v in data.items():
            # allow string keys like "roi1" or "1"
            m = re.search(r"(\d+)", str(k))
            if not m:
                logger.warning(
                    f"Skipping invalid key in expected counts file: {k}")
                continue
            roi_id = int(m.group(1))
            if not isinstance(v, int) or v < 0:
                logger.warning(
                    f"Skipping invalid value for ROI {roi_id} in expected counts file: {v}")
                continue
            new_counts[roi_id] = v
        if new_counts:
            EXPECTED_COUNTS = new_counts
            logger.info(f"✅ Loaded EXPECTED_COUNTS from file: {NUM_ROI_PATH}")
        else:
            logger.info(
                "ℹ️ No valid expected counts found in file, keeping defaults")
        return EXPECTED_COUNTS
    except Exception as e:
        logger.error(f"❌ Failed to load expected counts: {e}")
        return EXPECTED_COUNTS


def save_expected_counts_to_file(counts: dict):
    """Ensure folder, remove existing JSONs in NUM_ROI_FOLDER, and save counts as JSON."""
    ensure_folder_exists(NUM_ROI_FOLDER)
    # Remove any existing json files in the folder (as requested)
    clean_json_files(NUM_ROI_FOLDER)
    # Save to fixed filename
    try:
        with open(NUM_ROI_PATH, 'w', encoding='utf-8') as f:
            json.dump(counts, f, indent=4, ensure_ascii=False)
        logger.info(f"💾 Expected counts saved to {NUM_ROI_PATH}")
        return NUM_ROI_PATH
    except Exception as e:
        logger.error(f"❌ Failed to save expected counts to file: {e}")
        raise


def get_camera_device_path():
    """Return device path from CAMERA_SOURCE (accepts '/dev/videoX' or '0')."""
    src = str(CAMERA_SOURCE)
    if src.startswith("/dev/video"):
        return src
    try:
        idx = int(src)
        return f"/dev/video{idx}"
    except Exception:
        return src


def run_v4l2_cmd(args):
    """Run v4l2-ctl with device and return (success, output)."""
    device = get_camera_device_path()
    cmd = ['v4l2-ctl', '-d', device] + args
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return True, res.stdout.strip()
    except FileNotFoundError:
        return False, "v4l2-ctl not found"
    except subprocess.CalledProcessError as e:
        return False, (e.stderr or str(e)).strip()


def send_good_signal():
    """Kirim pulsa GOOD ke PLC via Coil 80 secara aman (open/close lokal)."""
    global client
    try:

        # Kirim pulsa: ON lalu OFF (banyak PLC butuh edge/pulse, bukan level latched)
        ok_on = client.write_multiple_coils(COIL_REJECT_OUTPUT, [True])
        if not ok_on:
            logger.warning(f"⚠️ Gagal set Coil {COIL_REJECT_OUTPUT} = True")
            return
        logger.info(f"📡 Coil {COIL_REJECT_OUTPUT} = True (GOOD pulse start)")
        time.sleep(0.1)  # 100 ms pulse (sesuaikan kebutuhan PLC)

        ok_off = client.write_multiple_coils(COIL_REJECT_OUTPUT, [False])
        if not ok_off:
            logger.warning(f"⚠️ Gagal set Coil {COIL_REJECT_OUTPUT} = False")
        else:
            logger.info(
                f"📡 Coil {COIL_REJECT_OUTPUT} = False (GOOD pulse end)")

    except Exception as e:
        logger.error(f"❌ Error kirim GOOD signal ke PLC: {e}")
# ---------------------------
# PLC monitor (runs in background)
# ---------------------------


async def trigger_predict_and_log(batch_id: int):
    """Dipanggil oleh sensor/PLC trigger; menjalankan proses prediksi internal."""
    try:
        # Kirim batch_id ke fungsi prediksi
        res = await run_prediction(batch_id=batch_id)
        logger.info(
            f"✅ Prediksi (internal) untuk Batch {batch_id} selesai dipicu sensor")
    except Exception as e:
        logger.error(
            f"❌ Error during sensor-triggered prediction (Batch {batch_id}): {e}")


async def run_prediction(batch_id: int):
    global model, areas, current_results, current_status, part_status_db, plc_db_lock
    ensure_folder_exists(RESULTS_FOLDER)
    ensure_folder_exists(NOT_GOOD_BASE_FOLDER)

    if model is None:
        raise RuntimeError("Models not initialized")
    if not areas:
        raise RuntimeError("No ROI coordinates available. Use /insert first.")
    try:
        logger.info("🚀 Memulai proses prediksi (internal)...")
        timestamp = datetime.now()

        clean_json_files(RESULTS_FOLDER)

        image_path = capture_image_from_camera()

        # Tidak menghapus NOT_GOOD_BASE_FOLDER di sini agar folder yang belum selesai dikirim tidak terhapus
        # Folder akan dihapus setelah semua file berhasil dikirim di report_results_async
        clean_folder(CROPPED_FOLDER)

        # Ambil battery_id lebih awal untuk folder khusus
        battery_id = get_next_battery_id_json()
        not_good_folder = os.path.join(
            NOT_GOOD_BASE_FOLDER, f"not_good_{battery_id}")
        ensure_folder_exists(not_good_folder)

        result_image, results_all, total_objects, roi_counts, roi_statuses, overall_status = process_image_with_roi(
            image_path, areas, model, not_good_folder
        )

        good_rois = sum(1 for status in roi_statuses.values()
                        if status["status"] == "GOOD")

        indikator_good_rois = int(good_rois)
        total_rois = len(roi_statuses)

        try:
            update_status_counter(overall_status)
        except Exception as e_count:
            logger.error(f"❌ Gagal memperbarui status counter: {e_count}")

        current_results = {
            "total_objects": total_objects,
            "roi_counts": roi_counts,
            "roi_statuses": roi_statuses,
            "overall_status": overall_status,
            "detailed_results": results_all,
            "image_path": OUTPUT_PATH,
            "not_good_folder": not_good_folder,
            "battery_id": battery_id
        }

        current_status = {
            "overall_status": overall_status,
            "total_objects": total_objects,
            "roi_counts": roi_counts,
            "roi_statuses": roi_statuses,
            "timestamp": time.time()
        }

        # Kirim sinyal GOOD ke PLC jika overall_status == "GOOD"
        # (menggunakan threshold dari roi_good_threshold.json via check_overall_status)
        logger.info(f"Jumlah Good ROIs: {indikator_good_rois}/{total_rois}, Overall Status: {overall_status}")
        if overall_status == "GOOD":
            send_good_signal()
            logger.info("✅ Kirim sinyal GOOD ke PLC (Coil 80)")
        else:
            logger.info("❌ Tidak kirim sinyal GOOD (status NOT GOOD)")

        with plc_db_lock:
            part_status_db[batch_id] = overall_status
        logger.info(
            f"PLC PREDICT: Batch {batch_id} -> {overall_status} (coil 80 updated)")

        if os.path.exists(TEMP_IMAGE_PATH):
            os.remove(TEMP_IMAGE_PATH)

        response_data = {
            "status": "success",
            "message": "Detection completed successfully",
            "total_objects": total_objects,
            "roi_counts": roi_counts,
            "roi_statuses": roi_statuses,
            "overall_status": overall_status,
            "result_image": OUTPUT_PATH,
            "timestamp": timestamp.isoformat(),
            "roi_total": len(areas),
            "detection_summary": {
                "good_rois": good_rois,
                "not_good_rois": total_rois - good_rois,
                "total_rois": total_rois,
                "total_detected_objects": total_objects
            },
            "battery_id": battery_id,
            "not_good_folder": not_good_folder
        }

        json_filepath = save_detection_results(response_data, timestamp)
        response_data["saved_json_path"] = json_filepath

        # Jadwalkan reporting dengan folder khusus
        try:
            asyncio.create_task(
                report_results_async(
                    overall_status, not_good_folder, OUTPUT_PATH, battery_id)
            )
            response_data["reporting_scheduled"] = True
        except Exception as e:
            logger.error(f"❌ Gagal jadwalkan reporting: {e}")
            response_data["reporting_scheduled"] = False
            response_data["reporting_error"] = str(e)

        return response_data
    except Exception as e:
        logger.error(f"❌ Error during prediction: {e}")
        raise


def safe_plc_reconnect(client: ModbusClient, max_retries: int = None) -> bool:
    """
    Safely reconnect to PLC with proper cleanup.
    Returns True if reconnect successful, False if max_retries exceeded.
    """
    retry_count = 0

    # 1. PENTING: Tutup koneksi lama terlebih dahulu
    try:
        if client.is_open():
            client.close()
            logger.info("🔌 Socket PLC lama ditutup")
        else:
            logger.info("ℹ️ Socket PLC sudah tertutup")
    except Exception as e_close:
        logger.warning(f"⚠️ Error saat close socket: {e_close}")

    # 2. Tunggu untuk memastikan OS release socket (TIME_WAIT state)
    time.sleep(0.5)

    # 3. Loop reconnect dengan backoff
    while True:
        retry_count += 1
        if max_retries and retry_count > max_retries:
            logger.error(f"❌ Gagal reconnect setelah {max_retries} percobaan")
            return False

        try:
            logger.info(
                f"🔄 Percobaan reconnect #{retry_count} ke PLC {PLC_IP}:{PLC_PORT}...")

            if client.open():
                logger.info(
                    f"✅ Berhasil reconnect ke PLC (percobaan #{retry_count})")

                # Update status global
                with plc_status_lock:
                    globals()["plc_connected"] = True
                    globals()["plc_last_ok_ts"] = time.time()

                return True
            else:
                logger.warning(
                    f"⚠️ Percobaan #{retry_count} gagal. Retry dalam 2s...")
                time.sleep(2)

        except Exception as e_conn:
            logger.error(
                f"❌ Exception saat reconnect (percobaan #{retry_count}): {e_conn}")
            time.sleep(2)


def plc_monitor(loop):
    """
    PLC loop (Revisi):
    - Sensor 1 (480) rising edge -> ambil foto & mulai prediction (batch increment).
    - Coil 81 (Heartbeat) tetap ON selama loop.
    - Coil 80 sekarang dikontrol langsung oleh fungsi run_prediction (bukan oleh Sensor 2).
    - Sensor 2 (481) tidak lagi digunakan.
    """
    global client
    client = ModbusClient(host=PLC_IP, port=PLC_PORT,
                          unit_id=UNIT_ID, auto_open=False, auto_close=False)
    try:
        with plc_status_lock:
            globals()["plc_monitor_running"] = True
            globals()["plc_connected"] = False

        logger.info(f"🔌 Koneksi awal ke PLC {PLC_IP}:{PLC_PORT}...")
        if not safe_plc_reconnect(client):
            logger.error("❌ Gagal koneksi awal ke PLC. Monitor stop.")
            return

        sensor_1_prev = False
        global counter_sensor_1, part_status_db, plc_db_lock, plc_stop_event

        while True:
            if plc_stop_event.is_set():
                logger.info("🛑 Stop signal diterima. Keluar loop PLC.")
                break
            try:
                # Heartbeat
                hb_write = client.write_multiple_coils(COIL_HEARTBEAT, [True])
                if not hb_write:
                    logger.warning("⚠️ Heartbeat gagal. Reconnect...")
                    with plc_status_lock:
                        globals()["plc_connected"] = False
                    if not safe_plc_reconnect(client):
                        logger.error("❌ Reconnect gagal permanen.")
                        break
                    continue

                # Baca hanya Sensor 1
                read_s1 = client.read_coils(COIL_SENSOR_1_FOTO, 1)
                if read_s1 is None:
                    logger.warning("⚠️ Gagal baca Sensor 1. Reconnect...")
                    with plc_status_lock:
                        globals()["plc_connected"] = False
                    if not safe_plc_reconnect(client):
                        logger.error("❌ Reconnect gagal permanen.")
                        break
                    continue

                with plc_status_lock:
                    globals()["plc_connected"] = True
                    globals()["plc_last_ok_ts"] = time.time()

                sensor_1_state = bool(read_s1[0])

                # Rising edge Sensor 1 -> batch baru + trigger predict
                if sensor_1_state and not sensor_1_prev:
                    with plc_db_lock:
                        counter_sensor_1 += 1
                        current_batch_id = counter_sensor_1
                        part_status_db[current_batch_id] = "PENDING"
                        save_plc_counters(counter_sensor_1, 0)  # S2 diset 0

                    logger.info(
                        f"⚡ S1 ON → Mulai Batch {current_batch_id} (PENDING)")
                    asyncio.run_coroutine_threadsafe(
                        trigger_predict_and_log(current_batch_id), loop)

                sensor_1_prev = sensor_1_state
                time.sleep(0.1)

            except Exception as e:
                logger.error(f"❌ Error loop PLC: {e}")
                with plc_status_lock:
                    globals()["plc_connected"] = False
                if not safe_plc_reconnect(client):
                    logger.error("❌ Reconnect gagal setelah exception.")
                    break
                time.sleep(1)

    finally:
        logger.info("PLC monitor berhenti. Matikan coil output...")
        try:
            if client.is_open():
                client.write_multiple_coils(COIL_HEARTBEAT, [False])
                # Coil 80 tidak dimatikan di sini jika ingin tetap latched; bisa aktifkan baris berikut bila ingin OFF saat shutdown:
                # client.write_multiple_coils(COIL_REJECT_OUTPUT, [False])
            client.close()
        except Exception as e:
            logger.error(f"Cleanup PLC error: {e}")
        with plc_status_lock:
            globals()["plc_connected"] = False
            globals()["plc_monitor_running"] = False
        logger.info("PLC socket ditutup.")


async def start_plc_monitor():
    """Start plc_monitor in a background thread (non-blocking)."""
    if not PLC_MONITOR_ENABLED:
        logger.info("PLC monitor disabled by env")
        return
    with plc_status_lock:
        if plc_monitor_running:
            logger.info("PLC monitor already running; skip start.")
            return
        plc_stop_event.clear()
    loop = asyncio.get_event_loop()
    t = threading.Thread(target=plc_monitor, args=(loop,), daemon=True)
    t.start()
    # simpan thread & flag
    global plc_monitor_thread
    with plc_status_lock:
        plc_monitor_thread = t
        globals()["plc_monitor_running"] = True
    logger.info("PLC monitor thread started")


def stop_plc_monitor(timeout: float = 5.0):
    """Signal PLC monitor thread to stop and wait for cleanup."""
    if not PLC_MONITOR_ENABLED:
        return
    global plc_monitor_thread
    with plc_status_lock:
        thread = plc_monitor_thread
        running = plc_monitor_running
    if not thread or not running:
        logger.info("PLC monitor thread not running; nothing to stop.")
        return

    logger.info("🛑 Mengirim sinyal berhenti ke PLC monitor thread...")
    plc_stop_event.set()
    thread.join(timeout)

    if thread.is_alive():
        logger.warning(
            "⚠️ PLC monitor thread masih berjalan setelah timeout %.1fs", timeout)
    else:
        logger.info("✅ PLC monitor thread berhenti dengan aman.")

    with plc_status_lock:
        plc_monitor_thread = None if not thread.is_alive() else thread
        globals()["plc_monitor_running"] = False
        globals()["plc_connected"] = False


@app.post("/camera_set")
async def camera_set(payload: dict):
    """
    Set camera parameters.
    Payload example:
      {
        "set_4k": true,
        "zoom": 120,
        "focus": 10,
        "auto_focus": false
      }
    """
    results = {"device": get_camera_device_path(), "actions": {}}

    run_v4l2_cmd([f'--set-fmt-video=width=3840,height=2160,pixelformat=MJPG'])

    # set zoom
    if "zoom" in payload:
        try:
            z = int(payload["zoom"])
            ok, out = run_v4l2_cmd(['-c', f'zoom_absolute={z}'])
            results["actions"]["zoom"] = {"ok": ok, "output": out}
            # Simpan jika berhasil
            if ok:
                save_camera_settings({"zoom_absolute": z})
        except Exception as e:
            results["actions"]["zoom"] = {"ok": False, "output": str(e)}

    # set focus
    if "focus" in payload:
        try:
            fval = int(payload["focus"])
            ok, out = run_v4l2_cmd(['-c', f'focus_absolute={fval}'])
            results["actions"]["focus"] = {"ok": ok, "output": out}
            # Simpan jika berhasil
            if ok:
                save_camera_settings({"focus_absolute": fval})
        except Exception as e:
            results["actions"]["focus"] = {"ok": False, "output": str(e)}

    # set auto focus
    if "auto_focus" in payload:
        try:
            af = 1 if bool(payload["auto_focus"]) else 0
            ok, out = run_v4l2_cmd(['-c', f'focus_automatic_continuous={af}'])
            results["actions"]["auto_focus"] = {"ok": ok, "output": out}
            # Simpan jika berhasil
            if ok:
                save_camera_settings({"focus_automatic_continuous": af})
        except Exception as e:
            results["actions"]["auto_focus"] = {"ok": False, "output": str(e)}

    await asyncio.sleep(0.5)

    # Return current state after attempts
    current = await camera_get()
    results["current"] = current
    return results


@app.get("/camera_get")
async def camera_get():
    """Get camera current/native resolution and ctrl values (zoom, focus, auto_focus)."""
    device = get_camera_device_path()
    info = {
        "device": device,
        # current format reported by driver (--get-fmt-video) or OpenCV fallback    # largest supported resolution from --list-formats-ext     # list of supported resolutions reported
        "current_resolution": None,
        "zoom": None,
        "focus": None,
        "auto_focus": None
    }

    # 1) PRIORITASKAN OpenCV untuk mendapat resolusi AKTIF yang sebenarnya
    global cap
    if cap is not None and getattr(cap, 'isOpened', lambda: False)():
        try:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w > 0 and h > 0:
                info["current_resolution"] = {"width": w, "height": h}
                logger.info(f"📹 Resolusi dari OpenCV (cap aktif): {w}x{h}")
        except Exception as e:
            logger.warning(f"⚠️ Gagal baca resolusi dari cap aktif: {e}")

    # 2) Fallback: Buka kamera sementara jika cap tidak tersedia
    if info["current_resolution"] is None:
        try:
            cap_temp = cv2.VideoCapture(device)
            # Tunggu kamera siap
            time.sleep(0.3)

            if cap_temp.isOpened():
                w = int(cap_temp.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap_temp.get(cv2.CAP_PROP_FRAME_HEIGHT))
                if w > 0 and h > 0:
                    info["current_resolution"] = {"width": w, "height": h}
                    logger.info(f"📹 Resolusi dari OpenCV (temp): {w}x{h}")
            cap_temp.release()
        except Exception as e:
            logger.warning(f"⚠️ Gagal baca resolusi via OpenCV temp: {e}")

    # 3) Fallback kedua: v4l2-ctl --get-fmt-video (bisa tidak akurat jika driver belum update)
    if info["current_resolution"] is None:
        ok, out = run_v4l2_cmd(['--get-fmt-video'])
        if ok and out:
            m = re.search(r"Width/Height\s*:\s*(\d+)\s*/\s*(\d+)", out)
            if m:
                info["current_resolution"] = {"width": int(
                    m.group(1)), "height": int(m.group(2))}
                logger.info(
                    f"📹 Resolusi dari v4l2-ctl: {m.group(1)}x{m.group(2)}")

    # 4) Get controls (zoom, focus, auto focus)
    ok, out = run_v4l2_cmd(['--get-ctrl=zoom_absolute'])
    if ok and out:
        m = re.search(r"(\d+)", out)
        info["zoom"] = int(m.group(1)) if m else out
    else:
        info["zoom"] = out if out else None

    ok, out = run_v4l2_cmd(['--get-ctrl=focus_absolute'])
    if ok and out:
        m = re.search(r"(\d+)", out)
        info["focus"] = int(m.group(1)) if m else out
    else:
        info["focus"] = out if out else None

    ok, out = run_v4l2_cmd(['--get-ctrl=focus_automatic_continuous'])
    if ok and out:
        m = re.search(r"(\d+)", out)
        info["auto_focus"] = int(m.group(1)) if m else out
    else:
        info["auto_focus"] = out if out else None

    return info


# ==== ENDPOINTS ====
@app.get("/")
async def root():
    return {"message": "Object Detection API", "status": "running"}


@app.get("/counters/reset")
async def reset_counters():
    global counter_sensor_1, counter_sensor_2, part_status_db
    with plc_db_lock:
        counter_sensor_1 = 0
        counter_sensor_2 = 0
        part_status_db.clear()
        try:
            save_plc_counters(counter_sensor_1, counter_sensor_2)
        except Exception as e:
            logger.error(f"❌ Gagal reset counter PLC: {e}")
            raise HTTPException(
                status_code=500, detail="Failed to reset counters")

    # Reset battery counter
    try:
        ensure_folder_exists(RESULTS_FOLDER)
        tmp_path = BATTERY_COUNTER_PATH + ".tmp"
        with open(tmp_path, 'w', encoding='utf-8') as f:
            json.dump({"counter": 0}, f)
        os.replace(tmp_path, BATTERY_COUNTER_PATH)
        logger.info("🔄 Battery counter direset ke 0")
    except Exception as e:
        logger.error(f"❌ Gagal reset battery counter: {e}")
        raise HTTPException(
            status_code=500, detail="Failed to reset battery counter")

    # Reset status counter (GOOD/NOT_GOOD)
    try:
        ensure_folder_exists(RESULTS_FOLDER)
        tmp_path = STATUS_COUNTER_PATH + ".tmp"
        with open(tmp_path, 'w', encoding='utf-8') as f:
            json.dump({
                "good": 0,
                "not_good": 0,
                "total_processed": 0,
                "last_updated": datetime.now().isoformat()
            }, f, indent=4)
        os.replace(tmp_path, STATUS_COUNTER_PATH)
        logger.info("🔄 Status counter (GOOD/NOT_GOOD) direset ke 0")
    except Exception as e:
        logger.error(f"❌ Gagal reset status counter: {e}")
        raise HTTPException(
            status_code=500, detail="Failed to reset status counter")

    logger.info(
        "🔄 Semua counter direset: S1, S2, Battery ID, dan Status Counter")
    return {
        "status": "success",
        "message": "All counters reset to 0 (S1, S2, Battery ID, Status Counter)",
        "counters": {
            "sensor_1": counter_sensor_1,
            "sensor_2": counter_sensor_2,
            "battery_id": 0,
            "good": 0,
            "not_good": 0,
            "total_processed": 0
        }
    }


async def camera_capture_loop():
    global cap, camera_active, original_frame
    consecutive_fail = 0
    max_fail_before_reconnect = 5
    try:
        while camera_active:
            if not ensure_camera_open():
                logger.error("Stop loop: kamera tidak bisa direconnect.")
                break

            ret, frame = cap.read()
            if not ret or frame is None:
                consecutive_fail += 1
                logger.warning(f"Frame read gagal (#{consecutive_fail}).")
                if consecutive_fail >= max_fail_before_reconnect:
                    logger.warning(
                        "Mencapai batas gagal frame. Coba reconnect kamera...")
                    ensure_camera_open()
                    consecutive_fail = 0
                await asyncio.sleep(0.05)
                continue

            consecutive_fail = 0
            original_frame = frame
            await asyncio.sleep(0)  # cooperative
    except Exception as e:
        logger.error(f"Loop capture error: {e}")
    finally:
        logger.info("Loop capture berhenti")


def start_camera_if_needed():
    global camera_active, capture_task
    if not camera_active:
        camera_active = True
        logger.info("Menyalakan kamera & memulai loop capture...")
        # Try initial open (non-fatal if gagal, loop akan coba lagi)
        ensure_camera_open()
        capture_task = asyncio.create_task(camera_capture_loop())


@app.get("/frame")
async def video_feed():
    """Streaming dengan overlay ROI."""
    global stream_clients

    start_camera_if_needed()
    stream_clients += 1

    async def gen():
        global stream_clients
        try:
            # tunggu frame pertama
            attempts = 100
            while original_frame is None and attempts > 0:
                await asyncio.sleep(0.05)
                attempts -= 1
            if original_frame is None:
                raise HTTPException(
                    status_code=500, detail="Tidak menerima frame dari kamera")

            while True:
                frame = original_frame
                if frame is None:
                    await asyncio.sleep(0.02)
                    continue

                vis = frame.copy()
                for i, area in enumerate(areas):
                    x1, y1, x2, y2 = map(
                        int, [area["x1"], area["y1"], area["x2"], area["y2"]])
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.putText(vis, f"ROI {i+1}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                ok, buf = cv2.imencode(".jpg", vis)
                if not ok:
                    await asyncio.sleep(0.01)
                    continue
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")
                await asyncio.sleep(0.03)
        finally:
            stream_clients -= 1
            # await stop_camera_if_no_clients()

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/frame_original")
async def video_feed_original(capture: bool = False):
    """Streaming mentah tanpa overlay. Dapat dipanggil dulu; otomatis menyalakan kamera."""
    global stream_clients

    if capture:
        if original_frame is None:
            raise HTTPException(
                status_code=500, detail="No frame available from camera")

        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"capture_{timestamp}.jpg"
            filepath = os.path.join(RESULTS_FOLDER, filename)

            ensure_folder_exists(RESULTS_FOLDER)
            cv2.imwrite(filepath, original_frame)

            logger.info(f"📸 Frame captured and saved: {filepath}")
            return JSONResponse({
                "status": "success",
                "message": "Frame captured successfully",
                "filepath": filepath,
                "filename": filename,
                "timestamp": timestamp
            })
        except Exception as e:
            logger.error(f"❌ Error capturing frame: {e}")
            raise HTTPException(
                status_code=500, detail=f"Failed to capture frame: {str(e)}")

    start_camera_if_needed()
    stream_clients += 1

    async def gen():
        global stream_clients
        try:
            attempts = 100
            while original_frame is None and attempts > 0:
                await asyncio.sleep(0.05)
                attempts -= 1
            if original_frame is None:
                raise HTTPException(
                    status_code=500, detail="Tidak menerima frame dari kamera")

            while True:
                frame = original_frame
                if frame is None:
                    await asyncio.sleep(0.02)
                    continue
                ok, buf = cv2.imencode(".jpg", frame)
                if not ok:
                    await asyncio.sleep(0.01)
                    continue
                jpeg_bytes = buf.tobytes()
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg_bytes + b"\r\n")
                await asyncio.sleep(0.03)
        finally:
            stream_clients -= 1
            # await stop_camera_if_no_clients()

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.post("/debug_trigger")
async def debug_trigger(batch_id: int | None = None):
    """
    Debug endpoint to manually trigger detection, without disabling PLC trigger.
    - If batch_id is provided, it uses that ID.
    - If not provided, it will increment the S1 counter and create a new batch.
    Returns the same response structure as run_prediction.
    """
    global counter_sensor_1, part_status_db

    try:
        # Determine batch_id
        if batch_id is None:
            with plc_db_lock:
                counter_sensor_1 += 1
                batch_id = counter_sensor_1
                part_status_db[batch_id] = "PENDING"
                save_plc_counters(counter_sensor_1, counter_sensor_2)
            logger.info(f"🧪 Debug trigger: created Batch {batch_id} (PENDING)")
        else:
            # Ensure db entry exists as PENDING if not already
            with plc_db_lock:
                if batch_id not in part_status_db:
                    part_status_db[batch_id] = "PENDING"
            logger.info(f"🧪 Debug trigger: using Batch {batch_id} (PENDING)")

        # Run prediction and return the same payload
        response_data = await run_prediction(batch_id=batch_id)
        return JSONResponse(content={
            "status": "success",
            "message": f"Debug detection triggered for Batch {batch_id}",
            "batch_id": batch_id,
            **response_data
        })
    except Exception as e:
        logger.error(f"❌ Debug trigger error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/insert")
async def insert_roi(coordinates: ROICoordinates):
    """Endpoint untuk menambahkan/mengupdate ROI coordinates"""

    # Pastikan folder roi_coordinate ada
    ensure_folder_exists(os.path.dirname(JSON_PATH))
    clean_folder(ROI_JSON_FOLDER)
    try:
        # Konversi ke format yang diinginkan
        roi_list = [{"x1": roi.x1, "y1": roi.y1, "x2": roi.x2, "y2": roi.y2}
                    for roi in coordinates.coordinates]

        # Simpan ke file
        save_roi_coordinates(roi_list)

        # Update areas global
        global areas
        areas = roi_list

        return {
            "status": "success",
            "message": f"Successfully saved {len(roi_list)} ROI coordinates",
            "coordinates": roi_list
        }

    except Exception as e:
        logger.error(f"Error inserting ROI: {e}")
        raise HTTPException(
            status_code=500, detail=f"Failed to save ROI: {str(e)}")


@app.get("/roi")
async def get_roi():
    """Endpoint untuk mendapatkan ROI coordinates saat ini"""
    return {"coordinates": areas}


@app.post("/num_roi")
async def set_expected_counts(payload: dict):
    """
    Update expected object counts per ROI.

    Accepts payload like:
      {"1": 5, "2": 4}
    or
      {"roi1": 5, "roi2": 4}

    Keys will be parsed for digits; values must be non-negative integers.

    Saves the expected counts into num_roi/expected_counts.json.
    Existing JSON files in num_roi/ will be removed before saving (per requirement).
    """
    global EXPECTED_COUNTS
    try:
        new_counts = {}
        for raw_k, v in payload.items():
            # extract digits from key (supports "roi1", "ROI2", or "1")
            m = re.search(r"(\d+)", str(raw_k))
            if not m:
                raise HTTPException(
                    status_code=400, detail=f"Invalid ROI key: {raw_k}")
            roi_id = int(m.group(1))
            if not isinstance(v, int) or v < 0:
                raise HTTPException(
                    status_code=400, detail=f"Invalid expected count for ROI {roi_id}: {v}")
            new_counts[roi_id] = v

        if not new_counts:
            raise HTTPException(
                status_code=400, detail="No valid ROI counts provided")

        # Remove existing files and save new counts to disk
        save_expected_counts_to_file(new_counts)

        # Update in-memory EXPECTED_COUNTS
        EXPECTED_COUNTS = new_counts
        logger.info(
            f"✅ EXPECTED_COUNTS updated (memory + file): {EXPECTED_COUNTS}")

        # Kembalikan hasil terurut secara numerik agar klien melihat ROI 1,2,3,...10 secara benar
        sorted_expected = {
            str(k): EXPECTED_COUNTS[k] for k in sorted(EXPECTED_COUNTS.keys())}
        return {"status": "success", "expected_counts": sorted_expected}

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error updating expected counts: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health")
async def health_check():
    """Endpoint untuk memeriksa status sistem dan kamera"""
    camera_status = True if camera_active and cap is not None and cap.isOpened() else False
    # plc_status boolean saja
    with plc_status_lock:
        plc_bool = bool(plc_connected)
    return {
        "camera_status": camera_status,
        "plc_status": plc_bool,
    }


def _get_plc_status():
    """Build PLC status dict for health endpoint."""
    with plc_status_lock:
        enabled = PLC_MONITOR_ENABLED
        running = plc_monitor_running
        connected = plc_connected
        ts = plc_last_ok_ts
    last_ok_iso = datetime.fromtimestamp(ts).isoformat() if ts else None
    last_ok_age = round(time.time() - ts, 3) if ts else None
    return {
        "enabled": bool(enabled),
        "monitor_running": bool(running),
        "connected": bool(connected),
        "last_ok": last_ok_iso,
        "last_ok_age_sec": last_ok_age,
        "ip": PLC_IP,
        "port": PLC_PORT,
        "unit_id": UNIT_ID
    }


@app.get("/cleanup_roi")
async def cleanup_roi():
    clean_folder(ROI_JSON_FOLDER)
    global areas
    areas = []
    return {"status": "success", "message": "ROI JSON folder cleaned"}

# untuk reset model


@app.post("/reset_models")
async def reset_models():
    """Endpoint untuk me-reload models"""
    success = initialize_models()
    if success:
        return {"status": "success", "message": "Models reloaded successfully"}
    else:
        raise HTTPException(status_code=500, detail="Failed to reload models")


# ==== RUN APPLICATION ====
if __name__ == "__main__":
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
