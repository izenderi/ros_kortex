from pythonping import ping

def ping_host(host):
    ping_result = ping(target=host, size=1024, count=1000, timeout=2)

    return {
        'host': host,
        'avg_latency': ping_result.rtt_avg_ms,
        'min_latency': ping_result.rtt_min_ms,
        'max_latency': ping_result.rtt_max_ms,
        'packet_loss': ping_result.packet_loss
    }

hosts = [
    '10.13.146.101',
    '169.235.25.145',
    'google.com'
]

for host in hosts:
    print(ping_host(host))

#================================================================================

