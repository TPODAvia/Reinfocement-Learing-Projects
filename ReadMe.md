https://github.com/tensorflow/tensorflow/issues/45930


The error message you're seeing is due to the PowerShell execution policy on your system, which is preventing scripts from running. PowerShell has a feature called "execution policies" that controls the conditions under which PowerShell loads configuration files and runs scripts. This feature helps prevent the execution of malicious scripts [Source 3](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_execution_policies?view=powershell-7.3).

To solve this issue, you can change the execution policy. Here are a few methods you can use:

1. **Temporarily bypass the execution policy**: This method only solves the issue temporarily for the current session. You can use the following command to bypass the execution policy:

```powershell
powershell -ExecutionPolicy Bypass -File script.ps1
```

Replace `script.ps1` with the path to your script [Source 2](https://lazyadmin.nl/powershell/running-scripts-is-disabled-on-this-system/).

2. **Change the execution policy for all users on the computer**: You can change the policy for all users on your computer using the following command. Note that you need to have elevated permissions (Administrator permission) to do this:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope LocalMachine
```

This command sets the execution policy to `RemoteSigned` for all users on the computer [Source 2](https://lazyadmin.nl/powershell/running-scripts-is-disabled-on-this-system/).

3. **Change the execution policy only for the current PowerShell session**: If you need to run a few scripts without changing the policy permanently, you can change the execution policy for the current session only:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
```

This command sets the execution policy to `RemoteSigned` for the current PowerShell session [Source 2](https://lazyadmin.nl/powershell/running-scripts-is-disabled-on-this-system/).

Remember, you can check your current execution policy using the `Get-ExecutionPolicy` cmdlet [Source 5](https://www.makeuseof.com/enable-script-execution-policy-windows-powershell/).
