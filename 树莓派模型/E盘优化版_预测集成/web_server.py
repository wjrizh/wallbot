import os
import subprocess
import signal
import psutil
import json
import io
import zipfile
from flask import Flask, render_template, jsonify, send_from_directory, request, send_file

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
process = None

app = Flask(__name__)

# --- 配置区 ---
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_PY_PATH = os.path.join(PROJECT_DIR, 'main.py')
VENV_PYTHON = os.path.join(PROJECT_DIR, 'venv/bin/python3')
LOG_FILE = os.path.join(PROJECT_DIR, 'program.log')
PID_FILE = os.path.join(PROJECT_DIR, 'program.pid')

# 数据文件夹
DATA_DIR = os.path.join(PROJECT_DIR, 'data_storage')

# 确保数据文件夹存在
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)


# --- 工具函数 ---
def is_program_running():
    if not os.path.exists(PID_FILE):
        return False
    try:
        with open(PID_FILE, 'r') as f:
            pid = int(f.read().strip())
        return psutil.pid_exists(pid)
    except Exception:
        return False


def get_latest_logs(n=50):
    if not os.path.exists(LOG_FILE):
        return "暂无日志，请启动程序。"
    try:
        result = subprocess.check_output(['tail', '-n', str(n), LOG_FILE])
        return result.decode('utf-8', errors='replace')
    except Exception:
        return "读取日志失败。"


def get_file_list():
    """获取 data_storage 文件夹下的所有数据文件"""
    files = []
    if not os.path.exists(DATA_DIR):
        return files

    for filename in os.listdir(DATA_DIR):
        path = os.path.join(DATA_DIR, filename)

        # 只展示普通文件，排除可能误入的 .py 脚本
        if os.path.isfile(path) and not filename.endswith('.py'):
            size_bytes = os.path.getsize(path)
            original_size = size_bytes

            for unit in ['B', 'KB', 'MB', 'GB']:
                if size_bytes < 1024.0:
                    size_str = f"{size_bytes:.1f} {unit}"
                    break
                size_bytes /= 1024.0
            else:
                size_str = f"{size_bytes:.1f} TB"

            files.append({
                'name': filename,
                'size': size_str,
                'size_bytes': original_size,
                'mtime': os.path.getmtime(path)
            })

    files.sort(key=lambda x: x['mtime'], reverse=True)
    return files


def is_safe_filename(filename):
    """基础安全检查，防止路径穿越"""
    if not filename:
        return False
    if '..' in filename or filename.startswith('/'):
        return False
    if os.path.sep in filename:
        return False
    return True


# --- 网页路由 ---
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/info', methods=['GET'])
def get_info():
    status = 'running' if is_program_running() else 'stopped'
    return jsonify({'status': status, 'logs': get_latest_logs()})


@app.route('/api/start', methods=['POST'])
def start_program():
    if is_program_running():
        return jsonify({'message': '程序已在运行中。'})

    try:
        # 接收前端传来的参数并写入 config.json
        config_data = request.json
        if config_data:
            with open(os.path.join(BASE_DIR, "config.json"), "w", encoding="utf-8") as f:
                json.dump(config_data, f, ensure_ascii=False, indent=2)

        # 清空旧日志，准备记录新日志
        with open(LOG_FILE, 'w', encoding='utf-8') as f:
            f.write("")
        log_file_handle = open(LOG_FILE, 'a', encoding='utf-8')

        # 使用虚拟环境的 Python 启动，并实时输出日志
        process = subprocess.Popen(
            [VENV_PYTHON, '-u', MAIN_PY_PATH],
            stdout=log_file_handle,
            stderr=subprocess.STDOUT,
            cwd=BASE_DIR,
            preexec_fn=os.setsid
        )

        # 保存 PID 供停止功能使用
        with open(PID_FILE, 'w', encoding='utf-8') as f:
            f.write(str(process.pid))

        return jsonify({'message': '已应用最新参数并启动采集！'})
    except Exception as e:
        return jsonify({'message': f'启动失败: {str(e)}'}), 500


@app.route('/api/stop', methods=['POST'])
def stop_program():
    if not is_program_running():
        return jsonify({'message': '程序未在运行。'})
    try:
        with open(PID_FILE, 'r', encoding='utf-8') as f:
            pid = int(f.read().strip())
        os.killpg(os.getpgid(pid), signal.SIGTERM)
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        return jsonify({'message': '程序已成功停止。'})
    except Exception as e:
        return jsonify({'message': f'停止失败: {str(e)}'}), 500


@app.route('/api/files', methods=['GET'])
def list_files():
    return jsonify(get_file_list())


@app.route('/download/<path:filename>')
def download_file(filename):
    if not is_safe_filename(filename):
        return "非法请求", 403
    return send_from_directory(DATA_DIR, filename, as_attachment=True)


@app.route('/api/delete/<path:filename>', methods=['POST'])
def delete_file(filename):
    try:
        if not is_safe_filename(filename):
            return jsonify({'message': '非法操作'}), 403

        file_path = os.path.join(DATA_DIR, filename)
        if os.path.exists(file_path) and os.path.isfile(file_path):
            os.remove(file_path)
            return jsonify({'message': f'[{filename}] 已成功删除！'})
        return jsonify({'message': '文件不存在'}), 404
    except Exception as e:
        return jsonify({'message': f'删除失败: {str(e)}'}), 500


@app.route('/api/delete_selected', methods=['POST'])
def delete_selected_files():
    try:
        data = request.get_json(silent=True) or {}
        filenames = data.get('filenames', [])

        if not filenames:
            return jsonify({'message': '未选择任何文件'}), 400

        deleted = []
        skipped = []

        for filename in filenames:
            if not is_safe_filename(filename):
                skipped.append(filename)
                continue

            file_path = os.path.join(DATA_DIR, filename)
            if os.path.exists(file_path) and os.path.isfile(file_path):
                os.remove(file_path)
                deleted.append(filename)
            else:
                skipped.append(filename)

        return jsonify({
            'message': f'成功删除 {len(deleted)} 个文件，跳过 {len(skipped)} 个文件。',
            'deleted': deleted,
            'skipped': skipped
        })
    except Exception as e:
        return jsonify({'message': f'批量删除失败: {str(e)}'}), 500


@app.route('/api/download_selected', methods=['POST'])
def download_selected_files():
    try:
        data = request.get_json(silent=True) or {}
        filenames = data.get('filenames', [])

        if not filenames:
            return jsonify({'message': '未选择任何文件'}), 400

        memory_file = io.BytesIO()
        added_count = 0

        with zipfile.ZipFile(memory_file, 'w', zipfile.ZIP_DEFLATED) as zf:
            for filename in filenames:
                if not is_safe_filename(filename):
                    continue

                file_path = os.path.join(DATA_DIR, filename)
                if os.path.exists(file_path) and os.path.isfile(file_path):
                    zf.write(file_path, arcname=filename)
                    added_count += 1

        if added_count == 0:
            return jsonify({'message': '没有可下载的有效文件'}), 400

        memory_file.seek(0)

        return send_file(
            memory_file,
            as_attachment=True,
            download_name='selected_files.zip',
            mimetype='application/zip'
        )
    except Exception as e:
        return jsonify({'message': f'批量下载失败: {str(e)}'}), 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
