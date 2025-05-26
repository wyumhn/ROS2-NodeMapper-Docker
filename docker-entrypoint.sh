#!/bin/bash
set -e

echo "--- docker-entrypoint.sh 開始 ---"

# pm2 プロセスに登録
echo "PM2のホームディレクトリをクリーンアップし、再初期化します。"
rm -rf /root/.pm2 || true # PM2のホームディレクトリを強制削除
pm2 startup || true # PM2の環境を再初期化（エラーが出てもスクリプトを続行）
pm2 kill || true # 念のため、起動中のPM2デーモンを再度停止
echo "PM2クリーンアップ・再初期化完了。"

echo "PM2 Runtime をフォアグラウンドで実行し、エコシステムファイルでアプリケーションを起動します。"
exec pm2-runtime /root/ecosystem.config.js --json