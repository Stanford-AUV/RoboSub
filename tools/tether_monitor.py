#!/usr/bin/env python3

import subprocess
import time
import argparse
from typing import Optional
from datetime import datetime
from zoneinfo import ZoneInfo

# Checks that the Orin is alive and responds to SSH
# MUST INSTALL sshpass: `brew install sshpass`

def check_ssh_connection(host: str, username: str, password: str, timeout: int = 5) -> bool:
    """
    Check connection to host via SSH and return True if successful, False otherwise.
    
    Args:
        host: The hostname to connect to
        username: SSH username
        password: SSH password
        timeout: Timeout in seconds for the SSH command
    
    Returns:
        bool: True if SSH connection successful, False otherwise
    """
    try:
        # Using sshpass to handle password authentication
        result = subprocess.run(
            [
                "sshpass", "-p", password,
                "ssh",
                "-o", "StrictHostKeyChecking=no",  # Don't ask to verify host key
                "-o", f"ConnectTimeout={timeout}",  # Connection timeout
                f"{username}@{host}",
                "echo 'Connection test'"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, subprocess.SubprocessError):
        return False

def monitor_connection(host: str, username: str, password: str, interval: int = 5, max_failures: int = 5) -> None:
    """
    Monitor connection to a host with specified interval.
    
    Args:
        host: The host to monitor
        interval: Time between pings in seconds
        max_failures: Number of consecutive failures before raising an error
    """
    consecutive_failures = 0
    
    print(f"Starting connection monitor for {host}")
    print(f"Ping interval: {interval} seconds")
    print(f"Max consecutive failures before error: {max_failures}")
    
    while True:
        if check_ssh_connection(host, username, password):
            if consecutive_failures > 0:
                print(f"Connection restored to {host}")
            consecutive_failures = 0
            pacific_time = datetime.now(ZoneInfo('America/Los_Angeles'))
            print(f"[{pacific_time.strftime('%Y-%m-%d %H:%M:%S %Z')}] ✓ Successfully pinged {host}")
        else:
            consecutive_failures += 1
            pacific_time = datetime.now(ZoneInfo('America/Los_Angeles'))
            print(f"[{pacific_time.strftime('%Y-%m-%d %H:%M:%S %Z')}] ✗ Failed to ping {host} (Failure {consecutive_failures}/{max_failures})")
            
            if consecutive_failures >= max_failures:
                raise ConnectionError(
                    f"Lost connection to {host} after {max_failures} consecutive failed attempts"
                )
        
        time.sleep(interval)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor connection to a host")
    parser.add_argument(
        "--interval",
        type=int,
        default=5,
        help="Time between pings in seconds (default: 5)"
    )
    parser.add_argument(
        "--max-failures",
        type=int,
        default=5,
        help="Number of consecutive failures before raising error (default: 5)"
    )
    parser.add_argument(
        "--host",
        type=str,
        default="ubuntu.local",
        help="Host to monitor (default: ubuntu.local)")
    
    parser.add_argument(
        "--username",
        type=str,
        default="robosub",
        help="SSH username (default: robosub)")
        
    parser.add_argument(
        "--password",
        type=str,
        default="robosub2024",
        help="SSH password")
    
    args = parser.parse_args()
    
    try:
        monitor_connection(
            host=args.host,
            username=args.username,
            password=args.password,
            interval=args.interval,
            max_failures=args.max_failures
        )
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    except ConnectionError as e:
        print(f"\nError: {e}")
        exit(1)
