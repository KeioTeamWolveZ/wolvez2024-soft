import csv
import tkinter as tk
from tkinter import filedialog, messagebox

def convert_to_csv():
    # ファイル選択ダイアログを開く
    input_file_path = filedialog.askopenfilename(
        title="テキストファイルを選択してください",
        filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
    )
    
    if not input_file_path:
        messagebox.showinfo("キャンセル", "ファイル選択がキャンセルされました。")
        return

    # 出力CSVファイルのパスを設定
    output_file_path = filedialog.asksaveasfilename(
        title="保存先のCSVファイルを指定してください",
        defaultextension=".csv",
        filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
    )

    if not output_file_path:
        messagebox.showinfo("キャンセル", "保存先がキャンセルされました。")
        return

    try:
        # テキストファイルを読み込み、CSVファイルに書き込む
        with open(input_file_path, 'r', encoding='utf-8') as txt_file:
            with open(output_file_path, 'w', newline='', encoding='utf-8') as csv_file:
                csv_writer = csv.writer(csv_file)
                # ヘッダーを書き込む
                csv_writer.writerow(["time", "state", "releasing_state", "closing_state", "Time", "Lat", "Lng", "ax", "ay", "az", "q", "pressure", "cameraCount", "control_log1", "control_log2", "rv", "lv"])

                for line in txt_file:
                    # 行をカンマで分割し、リストとして取得
                    row = line.strip().split(',')
                    # 0	state:0	releasing_state:1	closing_state:1	Time:0	Lat:     0	Lng:     0	ax:-0.003	ay: 0.002	az:   0.0	q:   0.0	pressure: 99892	cameraCount:     0control_log1:      	control_log2:      	rv:     0	lv:     0　
                    # # コロンがあるとき、右側だけ取得する
                    for i in range(len(row)):
                        if ':' in row[i]:
                            row[i] = row[i].split(':')[-1]

                    # CSVに行を書き込む
                    csv_writer.writerow(row)

        messagebox.showinfo("完了", f"{output_file_path} に変換が完了しました。")

    except Exception as e:
        messagebox.showerror("エラー", f"エラーが発生しました: {e}")

# Tkinterのウィンドウを作成
root = tk.Tk()
root.title("TXT to CSV Converter")

# 変換ボタンを作成
convert_button = tk.Button(root, text="ファイルを選択してCSVに変換", command=convert_to_csv)
convert_button.pack(padx=20, pady=20)

# Tkinterのメインループを開始
root.mainloop()
