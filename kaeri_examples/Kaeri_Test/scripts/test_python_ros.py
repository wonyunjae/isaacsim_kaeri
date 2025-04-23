# Isaac Sim의 site-packages 디렉토리 찾기
ISAAC_SITE_PACKAGES=$(find ~/isaacsim -path "*/python/lib/python3.10/site-packages" | head -1)

# isaaclab_tasks에 대한 심볼릭 링크 생성
if [ -n "$ISAAC_SITE_PACKAGES" ]; then
    ln -s /home/smarthc/.local/lib/python3.10/site-packages/isaaclab_tasks $ISAAC_SITE_PACKAGES/
    echo "Created symlink in $ISAAC_SITE_PACKAGES"
fi