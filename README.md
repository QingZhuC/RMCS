
克隆仓库，注意需要使用 `recurse-submodules` 以克隆子模块：

```bash
git clone --recurse-submodules https://github.com/QingZhuC/RMCS.git
```

在 VSCode 中打开仓库：

```bash
code ./RMCS
```

进入docker容器后，在 VSCode 中新建终端，输入：

```bash
cp .vscode/settings.default.json .vscode/settings.json
```


在 VSCode 终端中输入：

```bash
build-rmcs
```

编写代码并编译完成后，可以使用：

```bash
set-robot mycomponents
launch-rmcs
```

在本机上运行代码。可实现输出 $sin(\omega t)$ ， $cos(\omega t)$ 和 $sin(\omega t) + cos(\omega t)$ 的可视化结果（通过foxglove查看）

