python -m pip install --upgrade pip
python -m pip install pyinstaller -r "$PSScriptRoot\..\requirements.txt"
pyinstaller --noconfirm "$PSScriptRoot\..\launcher.spec"
