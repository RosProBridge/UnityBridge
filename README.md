# ProBridge Package

This documentation provides a comprehensive guide on how to integrate the ProBridge package, which contains essential scripts for connecting simulators to ROS (Robot Operating System), into your Unity project.

## Prerequisites

Before you begin, ensure you have the following:

- Git installed on your computer.
- Unity Editor 2022.3 or later installed and ready to use.

## Installation Guide

### 1. Cloning the Repository

First, you need to clone the ProBridge package repository from the given URL. Open your terminal or command prompt and execute the following command:

```bash
git clone https://g.a-r-s.dev/Unity/ProBridge.git
```

This command clones the repository into your local machine. Ensure you note the location where the repository is cloned.

### 2. Accessing Package Manager in Unity

Next, you need to open Unity and access the Package Manager:

- Launch Unity and open your project.
- Go to `Window` > `Package Manager` to open the Package Manager window.

![Package Manager Guide](Images/pm.png "Guide to Access Package Manager in Unity")

### 3. Adding Package from Disk

Within the Package Manager:

- Click the `+` button in the top left corner.
- Select `Add package from disk...`.

![Package Manager Add](Images/pm_add.png "Adding a Package from Disk")

### 4. Selecting the package.json File

Navigate to the location where you cloned the ProBridge package repository, and select the `package.json` file. This action imports the ProBridge package into your Unity project.

### 5. Verifying Installation

If the installation process was successful, you should now be able to see `ProBridge` listed among the installed packages in the Package Manager. This confirms the package is ready for use in your project.

## Adding Updates

To add updates to the package, follow these steps:

### 1. Complete the Installation Process

Make sure you have successfully completed the installation steps mentioned above.

### 2. Editing Scripts

- In the Unity Editor, navigate to the `Packages` folder.
- Locate the `ProBridge` package and open it to view all the scripts.

![Package Folder](Images/packages_folder.png "Navigating to the ProBridge Package")

### 3. Making Changes

You can now edit or add new scripts as necessary for your project within the ProBridge package.

### 4. Applying Updates Locally

After making changes, these modifications will be present in your local copy of the Simulation repository. Any local project using this repository will automatically receive these updates.

### 5. Pushing Updates

Finally, you can push your changes through the Simulation repository to share the updates with everyone. This ensures that all projects using the ProBridge package benefit from the latest improvements.
