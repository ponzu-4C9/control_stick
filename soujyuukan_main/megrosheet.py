import numpy as np
import matplotlib.pyplot as plt
import os

def fit_polynomial(x, y, degree):
    """
    与えられた (x, y) のデータセットから n次多項式で近似し、その係数を計算します。
    """
    # np.polyfitを使用して最小二乗法で多項式近似の係数を取得
    # 係数は降べきの順 (a*x^n + b*x^(n-1) + ... + c)
    coefficients = np.polyfit(x, y, degree)
    
    # 係数から多項式オブジェクトを作成 (評価に便利)
    poly_func = np.poly1d(coefficients)
    
    return coefficients, poly_func

def main():
    # 1. input.csv からデータの読み込み
    csv_file = 'input.csv'
    
    if not os.path.exists(csv_file):
        print(f"エラー: {csv_file} が見つかりません。")
        return
    
    x_data = []
    y_data = []
    
    try:
        with open(csv_file, 'r', encoding='utf-8') as f:
            # 1行目を読み込んでヘッダーを取得
            header_line = f.readline().strip()
            # カンマまたはタブ・スペースで区切られている場合に対応
            header_line = header_line.replace(',', ' ')
            headers = header_line.split()
            
            # xとyの列が存在するか確認し、インデックスを取得
            if 'x' in headers and 'y' in headers:
                x_idx = headers.index('x')
                y_idx = headers.index('y')
                print(f"ヘッダーから 'x' (列インデックス: {x_idx}) と 'y' (列インデックス: {y_idx}) の列を正常に発見しました。")
            else:
                print("エラー: ヘッダーに 'x' または 'y' が見つかりません。")
                return
            
            # データ行の読み込み
            for line_num, line in enumerate(f, start=2):
                line = line.replace(',', ' ')
                parts = line.split()
                if not parts:
                    continue # 空行はスキップ
                
                # 指定したインデックス以上の要素がない行はスキップ
                if len(parts) <= max(x_idx, y_idx):
                    continue
                
                try:
                    x_val = float(parts[x_idx])
                    y_val = float(parts[y_idx])
                    x_data.append(x_val)
                    y_data.append(y_val)
                except ValueError:
                    print(f"警告: {line_num}行目のデータを数値に変換できませんでした。スキップします。")
                    
        x_data = np.array(x_data)
        y_data = np.array(y_data)
        print(f"{csv_file} から {len(x_data)} 件の有効なデータを読み込みました。")
        
    except Exception as e:
        print(f"データの読み込み中にエラーが発生しました: {e}")
        return
    
    # 2. 多項式近似の実行
    degree = 5  # 指定の通り、5次多項式で近似
    
    print(f"--- {degree}次多項式でデータを近似 ---")
    coefficients, poly_func = fit_polynomial(x_data, y_data, degree)
    
    print("近似された係数 (高次から順):")
    for i, coef in enumerate(coefficients):
        print(f"  x^{degree - i} の係数: {coef:.4f}")
        
    print("\n生成された多項式:")
    print(poly_func)
    
    # 3. 結果の可視化 (Matplotlib)
    # 元データを散布図として描画
    plt.scatter(x_data, y_data, label='Original Data (input.csv)', color='blue', alpha=0.5, s=10)
    
    # 近似曲線をプロットするための滑らかなx座標の配列を作成
    x_fit = np.linspace(min(x_data), max(x_data), 500)
    y_fit = poly_func(x_fit)
    
    # 近似曲線を描画
    plt.plot(x_fit, y_fit, label=f'Fitted Polynomial (degree={degree})', color='red', linewidth=2)
    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title(f'Polynomial Regression (n={degree})')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
