template<typename TPacket>
void P2PActionPacketAdapter<TPacket>::Commit(bool guarantee_delivery) {
  action_handler_->p2p_stream().output().Commit(packet_view_.priority(), guarantee_delivery);
}

template<typename TPacket>
TPacket *P2PActionPacketAdapter<TPacket>::operator->() {
  return reinterpret_cast<TPacket *>(packet_view_.content() + sizeof(P2PApplicationPacketHeader));
}

template<typename TRequest, typename TReply, typename TProgress>
const TRequest &P2PActionHandler<TRequest, TReply, TProgress>::GetRequest() const {
  return *reinterpret_cast<const TRequest *>(request_bytes());
}

template<typename TRequest, typename TReply, typename TProgress>
StatusOr<P2PActionPacketAdapter<TReply>> P2PActionHandler<TRequest, TReply, TProgress>::NewReply() {
  StatusOr<P2PMutablePacketView> maybe_packet = p2p_stream().output().NewPacket(request_priority());
  if (!maybe_packet.ok()) {
    return maybe_packet.status();
  }
  maybe_packet->length() = sizeof(P2PApplicationPacketHeader) + sizeof(TReply);
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_packet->content());
  header->action = action();
  header->stage = P2PActionStage::kReply;
  header->request_id = request_id();
  return P2PActionPacketAdapter<TReply>(this, *maybe_packet);
}

template<typename TRequest, typename TReply, typename TProgress>
StatusOr<P2PActionPacketAdapter<TProgress>> P2PActionHandler<TRequest, TReply, TProgress>::NewProgress() {
  StatusOr<P2PMutablePacketView> maybe_packet = p2p_stream().output().NewPacket(request_priority());
  if (!maybe_packet.ok()) {
    return maybe_packet.status();
  }
  maybe_packet->length() = sizeof(P2PApplicationPacketHeader) + sizeof(TProgress);
  P2PApplicationPacketHeader *header = reinterpret_cast<P2PApplicationPacketHeader *>(maybe_packet->content());
  header->action = action();
  header->stage = P2PActionStage::kProgress;
  header->request_id = request_id();
  return P2PActionPacketAdapter<TProgress>(this, *maybe_packet);
}
