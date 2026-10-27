import os
import sys
import time
import queue
import threading
import functools
import json
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
from vallenae.io import PriDatabase, TraDatabase, HitRecord, TraRecord, ParametricRecord
from vallenae.features import acoustic_emission as ae_features
from waveline import SpotWave
from waveline.spotwave import TRRecord

print = functools.partial(print, flush=True)

HERE = Path(__file__).resolve().parent


def main():
    # =========================================================================
    # 1. 基础配置与准备
    # =========================================================================
    try:
        port = SpotWave.discover()[0]
        print(f"发现设备端口: {port}")
    except IndexError:
        print("未找到 SpotWave 设备，请检查 USB 连接！")
        return 2

    data_dir = Path(os.environ.get("AE_DATA_DIR", HERE / "data_storage")).resolve()
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    pridb_filename = str(data_dir / f"AE_Data_{timestamp}.pridb")
    tradb_filename = str(data_dir / f"AE_Data_{timestamp}.tradb")

    data_dir.mkdir(parents=True, exist_ok=True)
    print(f"数据目录: {data_dir}")
    print(f"将写入: {pridb_filename}")
    if os.path.exists(pridb_filename):
        os.remove(pridb_filename)
    if os.path.exists(tradb_filename):
        os.remove(tradb_filename)

    adc_step_volts = 1.74e-6
    tr_mv_factor = 0.00174
    print("已加载固定的硬件ADC分辨率: 1.7400 μV/step")

    # =========================================================================
    # 2. 读取配置参数并计算
    # =========================================================================
    settings = {
        "sample_rate": 2000000,
        "threshold_db": 0,
        "filter_hp_khz": 20,
        "filter_lp_khz": 300,
        "ddt_us": 100000,
        "pretrigger": 20,
        "preamp_gain": 0,
        "acq_duration_s": 20
    }

    config_path = HERE / "config.json"
    if config_path.is_file():
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                settings.update(json.load(f))
            print(f"已加载网页参数: {settings}")
        except Exception as e:
            print(f"读取配置失败，使用默认参数: {e}")

    decimation = max(1, int(2_000_000 / settings["sample_rate"]))
    samplerate = 2_000_000 // decimation
    hp_filter = int(settings["filter_hp_khz"] * 1000) if settings["filter_hp_khz"] > 0 else None
    lp_filter = int(settings["filter_lp_khz"] * 1000) if settings["filter_lp_khz"] > 0 else None
    feature_threshold = (10 ** (settings["threshold_db"] / 20)) * 1e-6
    pretrigger = int(settings["pretrigger"])
    preamp_gain = float(settings["preamp_gain"])
    acq_duration = float(settings["acq_duration_s"])

    print(f"当前阈值电压: {feature_threshold * 1e6:.1f} µV")
    print(f"采集时长: {acq_duration} 秒")

    data_queue = queue.SimpleQueue()

    # =====================================================================
    # 3. 后台数据库写入线程
    # =====================================================================
    def async_db_writer():
        with PriDatabase(pridb_filename, mode="rwc") as pridb, \
             TraDatabase(tradb_filename, mode="rwc") as tradb:

            # 恢复参数表初始化：这是你当前报错的关键修复
            pridb.connection().execute("""
                INSERT INTO ae_params (ID, SetupID, Chan, "ADC_µV", "ADC_TE", "ADC_SS")
                VALUES (?, ?, ?, ?, ?, ?)
            """, (1, 1, 1, 1.0, 1.0, 1.0))

            tradb.connection().execute("""
                INSERT INTO tr_params (ID, SetupID, Chan, "ADC_µV", "TR_mV")
                VALUES (?, ?, ?, ?, ?)
            """, (1, 1, 1, 1.0, tr_mv_factor))

            print("数据库初始化完成，后台准备接收数据...")
            local_trai_counter = 1

            while True:
                try:
                    record = data_queue.get(timeout=1.0)
                    if record is None:
                        break

                    data_volts = record.data
                    current_trai = record.trai if getattr(record, "trai", 0) > 0 else local_trai_counter
                    if current_trai == local_trai_counter:
                        local_trai_counter += 1

                    tra_record = TraRecord(
                        time=record.time,
                        channel=record.channel,
                        param_id=1,
                        pretrigger=pretrigger,
                        threshold=0,
                        samplerate=samplerate,
                        samples=len(data_volts),
                        data=data_volts,
                        trai=current_trai
                    )
                    tradb.write(tra_record)
                    #修改1：添加单位转换
                    # =====================================================================

                    peak_amp_volts = ae_features.peak_amplitude(data_volts)
                    peak_amp = 20 * np.log10(peak_amp_volts / 1e-6)   # 转换为 dBae
                    # =====================================================================
                    energy = ae_features.energy(data_volts, samplerate)
                    rms = ae_features.rms(data_volts)
                    counts = ae_features.counts(data_volts, feature_threshold)
                    duration = ae_features.duration(data_volts, feature_threshold, samplerate)
                    rise_time = ae_features.rise_time(data_volts, feature_threshold, samplerate)
                    signal_strength = ae_features.signal_strength(data_volts, samplerate)

                    hit_record = HitRecord(
                        time=record.time,
                        channel=record.channel,
                        param_id=1,
                        amplitude=peak_amp,
                        energy=energy,
                        signal_strength=signal_strength,
                        rms=rms,
                        duration=duration,
                        rise_time=rise_time,
                        counts=counts,
                        trai=current_trai,
                        threshold=feature_threshold
                    )
                    pridb.write_hit(hit_record)

                    param_rec = ParametricRecord(
                        time=record.time,
                        param_id=1
                    )
                    pridb.write_parametric(param_rec)

                except queue.Empty:
                    continue
                except Exception:
                    print("写入过程发生异常:")
                    traceback.print_exc()

        print("数据库写入完毕并安全关闭。")
                     #修改3：删除了daemon=True
                     # =====================================================================

    writer_thread = threading.Thread(target=async_db_writer)
    writer_thread.start()

    # =========================================================================
    # 4. 连接设备并开始主采集循环
    # =========================================================================
    print("\n正在建立与硬件的唯一连接...")
    with SpotWave(port) as sw:
        print("连接成功，正在下发配置指令...")
        sw.set_continuous_mode(True)
        sw.set_status_interval(0)
        sw.set_tr_enabled(True)

        sw.set_tr_decimation(decimation)
        sw.set_ddt(settings["ddt_us"])
        sw.set_filter(hp_filter, lp_filter)

        if hasattr(sw, "set_pretrigger"):
            sw.set_pretrigger(pretrigger)
            print(f"已设置 pretrigger: {pretrigger}")
        else:
            print("当前接口未找到 set_pretrigger，跳过 pretrigger 设置。")

        if hasattr(sw, "set_preamp_gain"):
            sw.set_preamp_gain(preamp_gain)
            print(f"已设置 preamp gain: {preamp_gain} dB")
        else:
            print("当前接口未找到 set_preamp_gain，跳过 preamp gain 设置。")

        print(f"\n▶️ 开始高速采集 ({acq_duration} 秒)...")
        start_time = time.monotonic()
        # 采集异常时打印堆栈；保留 E 盘版本的采集与写盘逻辑。
        try:
            for record in sw.acquire(raw=False):
                if isinstance(record, TRRecord):
                    data_queue.put(record)
                if time.monotonic() - start_time >= acq_duration:
                    break
        except KeyboardInterrupt:
            print("\n用户手动停止采集。")
        except Exception:
            traceback.print_exc()
        finally:
            print("正在清理硬件缓存...")


    print("硬件连接已关闭，正在等待数据落盘...")
    data_queue.put(None)
    writer_thread.join(timeout=120)
    if writer_thread.is_alive():
        print("写盘线程超时未结束，数据可能不完整")
        return 3

    try:
        os.sync()
    except Exception:
        pass

    try:
        psz = os.path.getsize(pridb_filename)
        tsz = os.path.getsize(tradb_filename)
    except OSError as e:
        print(f"落盘后无法读取文件大小: {e}")
        return 4
    print(f"✅ 落盘完成 pridb={psz}B tradb={tsz}B")
    if psz <= 0 or tsz <= 0:
        print("文件大小异常")
        return 5
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except Exception:
        traceback.print_exc()
        code = 1
    sys.exit(int(code or 0))
