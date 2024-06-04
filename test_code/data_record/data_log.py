import datetime
import time
import os

# フォルダパス
log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log_files")

# ファイル名
now = datetime.datetime.now()
filename = now.strftime("%Y%m%d_%H%M%S") + ".csv"

# ファイルパス
filepath = os.path.join(log_dir, filename)

# ファイルが存在しない場合は作成
if not os.path.exists(filepath):
    with open(filepath, "w") as f:
        f.write("timestamp\n")

# 5Hzで記録
print("\033[33m","start logging","\033[0m")
while True:
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")

    # ファイルに書き込み
    with open(filepath, "a") as f:
        f.write(f"{timestamp}\n")

    # 0.2秒待つ
    time.sleep(0.2)
   

