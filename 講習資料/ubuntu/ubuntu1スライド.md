---
marp: true
theme: gaia
size: 16:9
paginate: true
header: ubuntu1スライド.md
footer: © 2024 Altair
---
![bg right width:400px](../../images/altair.png) 

<br>
<br>
<br>

# **Ubuntu講習**
<br>



![width:40px](../../images/image.png)　Altairu

![width:40px](../../images/image-1.png)　＠Flying___eagle

---

# **Ubuntuとは？**

![width:500px](images/Ubuntu_logo.png)

- オープンソースのLinuxディストリビューション
- デスクトップ、サーバ、クラウドなど幅広い用途に対応
- **Canonical社**が開発、サポート


---

# **基本コマンド**

- **ls**: ディレクトリの内容を表示
  ```sh
  ls
  ```
- **cd**: ディレクトリを変更
  ```sh
  cd /path/to/directory
  ```
---

# **基本コマンド**

- **cp**: ファイルをコピーして貼り付け
  ```sh
  cp /path/to/sourcefile /path/to/destination/
  ```
  - **例**: `cp /home/user/file.txt /home/user/backup/`
  - 指定した`sourcefile`を`destination`ディレクトリにコピーします。

---
# **基本コマンド**

- **mv**: ファイルを移動または名前を変更
  ```sh
  mv oldname newname
  ```
  - **例**: `mv /home/user/file.txt /home/user/backup/file.txt`
  - ファイルを指定した場所に移動するか、名前を変更します。

---
# **基本コマンド**
- **rm**: ファイルを削除
  ```sh
  rm filename
  ```
  - **例**: `rm /home/user/file.txt`
  - 指定したファイルを削除します。

---
# **基本コマンド**
- **mkdir**: ディレクトリを作成
  ```sh
  mkdir newdirectory
  ```
  - **例**: `mkdir /home/user/newfolder`
  - 新しいディレクトリを作成します。

---
# **基本コマンド**

- **rmdir**: ディレクトリを削除
  ```sh
  rmdir directoryname
  ```
  - **例**: `rmdir /home/user/oldfolder`
  - 指定したディレクトリを削除します（空である必要があります）。

---
# **基本コマンド**

- **rm -r**: ディレクトリとその内容を再帰的に削除
  ```sh
  rm -r directoryname
  ```
  - **例**: `rm -r /home/user/oldfolder`
  - 指定したディレクトリとその中の全てのファイルおよびサブディレクトリを削除します。

---

# **基本コマンド**

- **touch**: 空のファイルを作成、または既存のファイルのタイムスタンプを更新
  ```sh
  touch filename
  ```
  - **例**: `touch /home/user/newfile.txt`
  - 指定した名前の空のファイルを作成します。

---

# **基本コマンド**

- **gedit**: テキストエディタを使用してファイルを編集
  ```sh
  gedit filename
  ```
  - **例**: `gedit /home/user/newfile.txt`
  - GNOMEデスクトップ環境のテキストエディタで指定したファイルを開きます。

---
# **基本コマンド**
* USB確認
```bash
ls -l /dev/serial/by-id/
```
* USB承認
```bash
sudo chmod 777 /dev/ttyUSB0
```
---

# **シェルスクリプトの例**

### 簡単なスクリプト

```sh
#!/bin/bash
echo "Hello, World!"
```

- **#!/bin/bash**: スクリプトの先頭に記述し、シェルの種類を指定
- **echo**: メッセージを表示

---

# **Pythonスクリプトの実行**

- Pythonスクリプトの拡張子は通常 **.py**
- 実行方法
  ```sh
  python3 scriptname.py
  ```

### 例: Hello Worldプログラム

```python
print("Hello, World!")
```

---

# **自作コマンドの作り方**

- シェルスクリプトを作成して実行可能にすることで、自作コマンドを作成できます。

### ステップ1: スクリプトを作成

- 任意のエディタでシェルスクリプトファイルを作成
  ```sh
  nano mycommand.sh
  ```
---

### ステップ2: スクリプトにコマンドを記述

- 例: `mycommand.sh`の内容
  ```sh
  #!/bin/bash
  echo "This is my custom command!"
  ```

### ステップ3: スクリプトを実行可能にする

- 実行権限を追加
  ```sh
  chmod +x mycommand.sh
  ```
---

### ステップ4: スクリプトをパスに追加

- `/usr/local/bin`などにコピー
  ```sh
  sudo cp mycommand.sh /usr/local/bin/mycommand
  ```

### ステップ5: 自作コマンドを実行

- どこからでも実行可能
  ```sh
  mycommand
  ```

---
# 自作コマンド
 [Ubuntuで自作コマンド?を作ろう](https://qiita.com/_Altair_/items/10703229107ae8b6f6fa)
Qiitaの記事

---

## スクリプトファイルの作成
まずは、スクリプトファイルを作成。
altairって名前のコマンドにします。

```bash
nano ~/bin/altair
```
---

## スクリプトの内容を編集

```bash
#!/bin/bash
# altairコマンド

function show_help {
    echo "使用法: altair [オプション]"
    echo "オプション:"
    echo "  -h, --help      このヘルプメッセージを表示"
    echo "  txt             geditでサンプルのテキストファイルを開く"
    echo "  pdf [filename]  evinceでPDFファイルを開く"
    echo "  py [filename]   /home/user/Documents/python からPythonファイルを検索して実行する"
    echo "  mkpy [filename] /home/user/Documents/python にPythonファイルを作成してgeditで開く"
    echo "  oppy [filename] /home/user/Documents/python からPythonファイルを検索してgeditで開く"
}
```
---
## スクリプトの内容を編集

```bash
function open_text {
    gedit ~/sample.txt
}

function open_pdf {
    if [ -z "$1" ]; then
        echo "ファイル名を指定してや"
    else
        evince "$1"
    fi
}
```
---
## スクリプトの内容を編集

```bash

function run_python {
    if [ -z "$1" ]; then
        echo "ファイル名を指定してや"
    else
        python3 "/home/user/Documents/python/$1"
    fi
}

function make_python {
    if [ -z "$1" ]; then
        echo "ファイル名を指定してや"
    else
        touch "/home/user/Documents/python/$1"
        gedit "/home/user/Documents/python/$1"
    fi
}
```
---
## スクリプトの内容を編集

```bash
function open_python {
    if [ -z "$1" ]; then
        echo "ファイル名を指定してや"
    else
        gedit "/home/user/Documents/python/$1"
    fi
}
```
---
## スクリプトの内容を編集

```bash

case "$1" in
    -h|--help)
        show_help
        ;;
    txt)
        open_text
        ;;
    pdf)
        open_pdf "$2"
        ;;
    py)
        run_python "$2"
        ;;
    mkpy)
        make_python "$2"
        ;;
    oppy)
        open_python "$2"
        ;;
    *)
        echo "不正なオプションやで"
        show_help
        ;;
esac
```
---

## 実行権限を付与

作成したスクリプトに実行権限を付与

```bash
chmod +x ~/bin/altair
```

## PATHにディレクトリを追加

自作コマンドをどこからでも実行できるようにするために、スクリプトのディレクトリをPATHに追加。`~/.bashrc`に以下の行を追加。

```bash
echo 'export PATH=$PATH:~/bin' >> ~/.bashrc
source ~/.bashrc
```
---

## コマンドの実行

これで自作コマンド「altair」が完成。

### ヘルプを表示

```bash
altair -h
```

### テキストファイルを開く

```bash
altair txt
```
---

### PDFファイルを開く

```bash
altair pdf filename.pdf
```

### Pythonファイルを実行

```bash
altair py script.py
```
---

### Pythonファイルを作成して開く

```bash
altair mkpy new_script.py
```

### Pythonファイルを開く

```bash
altair oppy existing_script.py
```
