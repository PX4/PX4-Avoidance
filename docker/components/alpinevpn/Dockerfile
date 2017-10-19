FROM alpine

RUN apk update && \
	apk add openvpn iptables socat curl openssl

COPY entrypoint.sh /home/root/entrypoint.sh
RUN chmod +x /home/root/entrypoint.sh

EXPOSE 1194/udp

ENTRYPOINT /home/root/entrypoint.sh
