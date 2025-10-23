Codexエージェント協働仕様書  

---

## 1. プロジェクト概要

本プロジェクトは、ROS1で構築された自律走行ロボットシステムを基盤に、  
ROS2環境で再設計・再構築することを目的とした開発プロジェクトである。  

CodexはROS2版のコード群を中心に、人間開発者と協働して  
ノード生成・修正・設計書更新・テスト補助を行う。

---

## 2. ディレクトリ方針

```text
プロジェクトルート/
├── ros1_src/   # ROS1版 (参照専用)
└── ros2_src/   # ROS2版 (開発対象)
    ├── <packageA>/
    ├── <packageB>/
    └── <packageC>/
````

* `ros1_src/` : 参照専用。移植元の構成・パラメータ参照用。変更禁止。
* `ros2_src/` : Codexおよび開発者が開発対象とする領域。
  各サブディレクトリが独立したROS2パッケージとして構成される。

---

## 3. パッケージ構成規約（ROS2標準構成）

```text
<package_name>/
├── package.xml
├── setup.py
├── setup.cfg
├── CMakeLists.txt             # (msg/srvパッケージのみ)
├── resource/<package_name>/
├── launch/
│   └── <package>.launch.py
├── <package_name>/
│   ├── __init__.py
│   ├── <node>.py
│   └── *_core.py
├── docs/
│   ├── <package>_詳細設計書_phaseX.md
│   └── <package>_phaseX_検討記録.md
├── params/                    # 任意
├── tools/                     # 任意
├── tests/                     # 任意
│   └── test_*.py
└── (map/ or routes/)*         # 任意
```

---

## 4. 各ディレクトリの役割

| ディレクトリ                   | 内容         | Codexの扱い        |
| ------------------------ | ---------- | --------------- |
| `package.xml`, `setup.*` | パッケージ定義    | 依存関係を壊さない範囲で編集可 |
| `resource/`              | ament登録    | 自動生成必須          |
| `launch/`                | 実行構成定義     | launch単位で追加・修正可 |
| `<package_name>/`        | ノード・ロジック   | メイン開発対象。全行保持    |
| `docs/`                  | 設計書・検討記録   | コード変更と整合を保って更新  |
| `params/`, `tools/`      | 任意追加可      | 自由生成可           |
| `tests/`                 | pytest用テスト | 自由追加可（後述の注意参照）  |
| `map/`, `routes/`        | 実験データ      | 原則非編集領域         |

---

## 5. ファイル生成・更新ルール

1. **構成を崩さない**
2. **コメント・空行を保持**
3. **設計書整合を維持**
4. **命名規則**

| 種類     | 命名形式                        | 例                                |
| ------ | --------------------------- | -------------------------------- |
| ノード    | `<package>_node.py`         | `route_manager_node.py`          |
| コア処理   | `<package>_core.py`         | `follower_core.py`               |
| 詳細設計書  | `<package>_詳細設計書_phaseX.md` | `route_follower_詳細設計書_phase2.md` |
| 検討記録   | `<package>_phaseX_検討記録.md`  | `route_manager_phase2_検討記録.md`   |
| Launch | `<package>.launch.py`       | `route_planner.launch.py`        |
| テスト    | `test_*.py`                 | `test_route_manager_core.py`     |

---

## 6. Codexエージェント分担

| エージェント           | 担当範囲                      | 主なタスク           | 出力形式          |
| ---------------- | ------------------------- | --------------- | ------------- |
| **Codex(ROS2)**  | ノード・launch                | ノード修正／生成・QoS設定  | Python        |
| **Codex(Core)**  | `_core.py`                | ROS非依存処理・ロジック整備 | Python        |
| **Codex(Doc)**   | `docs/`                   | 設計書・検討記録更新      | Markdown      |
| **Codex(Build)** | `setup.py`, `package.xml` | 依存修正・登録         | diffまたは完全ファイル |
| **Codex(Test)**  | `tests/`                  | pytestスクリプト生成   | Python        |

---

## 7. 出力要件（Codexへの命令）

* 再生成時は同名で完全置換。
* テストは `tests/` 配下に作成すること。
* **Codexは `colcon build` や `ros2 run` などROS2環境依存コマンドを実行できない。**
  したがって、**テストは純粋なPython単体で実行できるもの（`pytest`など）に限定する。**
  ROSノードを直接起動するテストやROS通信を伴う統合試験は人間開発者が実施する。
* ROS1資産・実験データへの書き込み禁止。

---

## 8. コーディング規約

### 8.1 基本方針

* Google Python Style Guide に準拠し、**PEP8整形済み**で提出すること。
* コメント・docstringは**日本語で記述し、初見のエンジニアが処理の流れやアルゴリズムを理解できる粒度**とする。
* クラス名・関数名・変数名は統一した命名規則に従う。

---

### 8.2 Docstring形式

```python
def compute_distance(self, pose_a: Pose, pose_b: Pose) -> float:
    """2点間のユークリッド距離を算出する.

    Args:
        pose_a (Pose): 始点の位置姿勢.
        pose_b (Pose): 終点の位置姿勢.

    Returns:
        float: pose_aからpose_bまでの距離[m].
    """
```

---

### 8.3 命名規則

| 要素       | 規則                      | 例                                  |
| -------- | ----------------------- | ---------------------------------- |
| クラス      | PascalCase              | `RouteManagerCore`                 |
| 関数 / 変数  | snake_case              | `compute_distance()`               |
| 定数       | UPPER_CASE              | `STOP_DISTANCE_M = 1.5`            |
| ROSトピック名 | lower_case + スラッシュ区切り   | `/amcl_pose`, `/goal`              |
| ロガー      | `self.get_logger()` で統一 | `self.get_logger().info('起動しました')` |

---

### 8.4 型ヒント

* すべての関数・メソッドに型ヒントを付与する。
* 戻り値がない場合は `-> None` を明記する。

---

### 8.5 ログ出力方針

| レベル     | 用途      | 例                                          |
| ------- | ------- | ------------------------------------------ |
| `debug` | 内部状態の確認 | `self.get_logger().debug(f"index={i}")`    |
| `info`  | 通常の進行報告 | `self.get_logger().info("処理を開始しました")`      |
| `warn`  | 想定内の異常  | `self.get_logger().warn("再試行中です")`         |
| `error` | 明確な障害   | `self.get_logger().error("サービス呼出に失敗しました")` |
| `fatal` | 致命的停止   | `self.get_logger().fatal("致命的エラー発生")`      |

全ノードは「起動」「主要状態遷移」「終了」時に少なくとも `info` レベルでログを出すこと。

---

### 8.6 インポート順序

1. 標準ライブラリ
2. 外部ライブラリ（例：numpy）
3. ROS2 (`rclpy`, `geometry_msgs`, etc.)
4. 同パッケージ内モジュール

```python
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from route_manager.manager_core import ManagerCore
```

---

### 8.7 レイアウトと書式

* 1行あたり **最大100文字**。
* クラス間は空行2行、関数間は空行1行。
* コメントは文頭を大文字で始め、文末に句点を付ける。
* 日本語コメントでは文体を統一し、**処理の目的・背景・例外条件**を明記する。

---

### 8.8 テスト規約

* テストスクリプトは `tests/test_*.py` に配置。
* `pytest` 互換構文を使用。
* CodexはROS2環境を実行できないため、**Python単体で動作可能な関数単位のテストに限定する。**
* 依存最小で実行可能な単体テストを優先する。

```python
import pytest
from route_manager.manager_core import ManagerCore

def test_initial_state():
    """初期状態が 'idle' であることを確認する."""
    core = ManagerCore()
    assert core.state == "idle"
```

---

## 9. 出力言語ポリシー（Codex応答言語規定）

### 9.1 基本方針

* Codexのすべての出力（コード内コメント、docstring、設計書、検討記録、ログメッセージなど）は**日本語を優先**する。
* 英語は固有名詞、外部API仕様、ROSメッセージ名など**技術的に不可避な箇所のみ使用可**とする。
* 生成される出力は**日本語で明確に意図が伝わる粒度**を最優先とする。

### 9.2 コードコメント規定

* Codexが生成するPythonスクリプトのコメントおよびdocstringは、
  初見の日本語話者エンジニアが**処理の流れ・意図・アルゴリズムの選定理由**を理解できる粒度で記述する。

### 9.3 回答形式

* Codexの回答や提案（チャット出力）は、**説明・要約・警告などもすべて日本語で記述**する。
* READMEや外部仕様に英語が含まれる場合は、原文を保持しつつ日本語補足を併記する。

### 9.4 ドキュメント生成時の言語統一

* 設計書（`docs/`）および検討記録は原則日本語で記述。
* ファイル冒頭に「# 日本語文書」と明記し、英語との混在を避ける。

---

## 10. チャット出力言語ポリシー

* Codexおよび関連AIエージェントは、開発者とのすべてのやり取りを**日本語**で行う。
* 技術用語（API名・ROSメッセージ名・クラス名など）は英語表記を維持するが、
  説明・補足・提案はすべて日本語で表現する。
* 出力文体は以下の原則に従う：

  * 説明文：丁寧語・論理的構成
  * コード内コメント：簡潔で意図を明示する日本語
  * 設計書：常体（「〜する」「〜を行う」）で統一
