@echo off
cd /d "%~dp0"
if not exist venv (
  echo First run: creating the environment. This downloads about 200 MB, please wait.
  py -3.12 -m venv venv 2>nul || py -3.13 -m venv venv 2>nul || python -m venv venv
  if errorlevel 1 goto nopython
  venv\Scripts\python.exe -m pip install --upgrade pip
  venv\Scripts\python.exe -m pip install -r requirements.txt
  if errorlevel 1 goto badinstall
)
echo Starting STEP to URDF. Close this window to stop it.
venv\Scripts\python.exe -m streamlit run app.py
goto end
:nopython
echo.
echo Python was not found. Install Python 3.12 from python.org/downloads
echo and tick "Add python.exe to PATH" during setup.
goto end
:badinstall
echo.
echo Install failed. Most likely your Python is 3.14, which the CAD kernel
echo does not support yet. Install Python 3.12 and delete the venv folder,
echo then run this file again.
:end
pause
