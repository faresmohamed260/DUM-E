python -m pip install --upgrade pip
python -m pip install pyinstaller -r "$PSScriptRoot\..\requirements.txt"
python -m PyInstaller --clean --noconfirm "$PSScriptRoot\..\launcher.spec"
