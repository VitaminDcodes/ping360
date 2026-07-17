# Grant full permissions to the current folder and all subfolders on Windows
Write-Host "Granting full permissions to the current folder and all subfolders..."
icacls . /grant "Everyone:(OI)(CI)F" /T

# Create virtual environment if it doesn't exist
if (-not (Test-Path -Path "venv")) {
    Write-Host "Creating Python virtual environment..."
    python -m venv venv
}

# Install dependencies
Write-Host "Installing dependencies..."
.\venv\Scripts\python.exe -m pip install --upgrade pip
.\venv\Scripts\python.exe -m pip install -r requirements.txt

Write-Host "Environment setup complete!"
