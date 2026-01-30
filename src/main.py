import multiprocessing


if __name__ == '__main__':
    # Freeze Support for Windows
    multiprocessing.freeze_support()


    # NOTE: 現在ロボットに付いているツールが何かを管理する方法がないので
    # ロボット制御コードの使用者に指定してもらう
    # ツールによっては、ツールとの通信が不要なものがあるので、通信の成否では判定できない
    # 現在のツールの状態を常にファイルに保存しておき、ロボット制御コードを再起動するときに
    # そのファイルを読み込むようにすれば管理はできるが、エラーで終了したときに
    # ファイルの情報が正確かいまのところ保証できないので、指定してもらう
    import argparse
    parser = argparse.ArgumentParser()
    # NOTE: Doosanでは現状1つのツールに対応
    # parser.add_argument(
    #     "--tool-id",
    #     type=int,
    #     required=True,
    #     choices=tool_ids,
    #     help="現在ロボットに付いているツールのID",
    # )
    parser.add_argument(
        "--use-joint-monitor-plot",
        action="store_true",
        help="関節角度のモニタープロットを使用する",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="GUIを使用する",
    )
    args = parser.parse_args()
    kwargs = vars(args)
    import os
    # HACK: コードの変化を少なくするため、
    # ロボット制御プロセスに引数で渡すのではなく環境変数で渡す
    # os.environ["TOOL_ID"] = str(kwargs.pop("tool_id"))
    # NOTE: 現状URではツールIDは1固定
    os.environ["TOOL_ID"] = "1"
    if kwargs.pop("gui"):   
        import tkinter as tk
        from doosan.gui import MQTTWin
        root = tk.Tk()
        mqwin = MQTTWin(root, **kwargs)
        mqwin.root.lift()
        root.protocol("WM_DELETE_WINDOW", mqwin.on_closing)
        root.mainloop()
    else:
        from doosan.headless import HeadlessLoop
        headless_loop = HeadlessLoop(**kwargs)
        headless_loop.mainloop()
