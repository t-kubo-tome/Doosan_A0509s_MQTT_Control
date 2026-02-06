import argparse
import multiprocessing


if __name__ == '__main__':
    # Freeze Support for Windows
    multiprocessing.freeze_support()

    parser = argparse.ArgumentParser()

    # NOTE: 現在ロボットに付いているツールが何かを管理する方法がないので
    # ロボット制御コードの使用者に指定してもらう
    # ツールによっては、ツールとの通信が不要なものがあるので、通信の成否では
    # 現在ロボットに付いているツールが何かを判定できない
    # 現在のツールの状態を常にファイルに保存しておき、ロボット制御コードを再起動するときに
    # そのファイルを読み込むようにすれば管理はできるが、エラーで終了したときに
    # ファイルの情報が正確かいまのところ保証できないので、指定してもらうことにしている
    # 管理する方法が確立されたら変更される可能性あり
    # ツールが1つの場合は1、2つ以上の場合はコマンドラインでの指定が必須
    from doosan.doosan_tools import tool_infos
    parser.add_argument(
        "--tool-id",
        type=int,
        required=len(tool_infos) > 1,
        choices=[tool_info["id"] for tool_info in tool_infos],
        default=1,
        help="現在ロボットに付いているツールのID",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="GUIを使用する",
    )
    parser.add_argument(
        "--use-joint-monitor-plot",
        action="store_true",
        help="関節角度のモニタープロットを使用する。GUIを使用する場合のみ指定可能",
    )
    args = parser.parse_args()
    kwargs = vars(args)

    # HACK: 現在ロボットに付いているツールが何かを管理する方法が確立されたら
    # コードが変更される可能性があるため、変更しやすいように引数で渡すのではなく
    # 環境変数でグローバル変数として使う
    import os
    os.environ["TOOL_ID"] = str(kwargs.pop("tool_id"))

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
