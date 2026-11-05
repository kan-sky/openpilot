"""Optional paired LAN maintenance; never part of modeld's realtime path."""
from tools.jetlink.manage_jetson import daemon


def main():
  daemon()


if __name__ == '__main__':
  main()
