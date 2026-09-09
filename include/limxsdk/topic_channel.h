/**
 * @file topic_channel.h
 *
 * @brief generic_sub_pub_topic channel: subscribe to any mros topic through a generated
 *        message mirror, without exposing mros to the integrator.
 *
 * The channel is split in two deliberately:
 *
 *  * ``openSub`` / ``closeSub`` / ``openPub`` / ``publishRaw`` / ``closePub`` are
 *    the only things that cross into the shared library. Their signatures carry
 *    no mros type and no template parameter, so the exported surface can be
 *    frozen once and stay stable while message types keep being added.
 *  * ``subscribe<M>`` / ``advertise<M>`` live entirely in this header. They encode
 *    and decode with the generated codec and never expose mros.
 *
 * Delivery follows the same contract as the existing ``subscribeXxx`` interfaces:
 * every frame is a freshly allocated object that is never mutated afterwards, so
 * the callee may keep it, share it, and read it from another thread.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_TOPIC_CHANNEL_H_
#define _LIMX_SDK_TOPIC_CHANNEL_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "limxsdk/codec/codec.h"
#include "limxsdk/macros.h"

namespace limxsdk
{
  namespace channel
  {
    /**
     * @brief Raw frame sink. @p data points at the payload with the mros caller
     *        id already stripped, and is only valid for the duration of the call.
     */
    typedef void (*RawFrameCallback)(const unsigned char *data, int size, void *user);

    /**
     * @brief Create a subscription described purely by its wire identity.
     *
     * @param topic      Topic name.
     * @param type       ROS type name, e.g. "controller_msgs/JointState".
     * @param md5        Message md5. mros matches a subscription on type + md5;
     *                   a mismatch yields a subscription that never fires.
     * @param definition ROS concatenated definition. Not used for matching, but
     *                   reported to peers and tooling.
     * @param cb         Invoked once per received frame, on the mros dispatch
     *                   thread for this topic. Must not block.
     * @param user       Opaque pointer handed back to @p cb.
     * @return An opaque handle, or nullptr on bad arguments.
     */
    LIMX_SDK_API void *openSub(const char *topic,
                               const char *type,
                               const char *md5,
                               const char *definition,
                               RawFrameCallback cb,
                               void *user);

    /**
     * @brief Tear down a subscription created by openSub().
     *
     * On return @p cb is guaranteed not to be running and not to be called
     * again, so the caller may free whatever @p user pointed at. Calling this
     * from inside the callback itself is not supported.
     */
    LIMX_SDK_API void closeSub(void *handle);

    /**
     * @brief Create a publisher described purely by its wire identity.
     *
     * @param topic      Topic name.
     * @param type       ROS type name, e.g. "controller_msgs/JointCmd".
     * @param md5        Message md5. mros matches a subscriber on type + md5;
     *                   a mismatch yields a publisher whose frames are silently
     *                   dropped by the peer.
     * @param definition ROS concatenated definition. Not used for matching, but
     *                   reported to peers and tooling.
     * @return An opaque handle, or nullptr on bad arguments.
     */
    LIMX_SDK_API void *openPub(const char *topic,
                               const char *type,
                               const char *md5,
                               const char *definition);

    /**
     * @brief Publish one already-encoded frame on a publisher from openPub().
     *
     * @p data is copied by mros before this returns, so the caller may free it
     * immediately. A zero-length frame (@p size == 0) is valid (std_msgs/Empty).
     *
     * @return 0 on success, -1 if @p handle is not an open publisher.
     */
    LIMX_SDK_API int publishRaw(void *handle, const unsigned char *data, int size);

    /// @brief Tear down a publisher created by openPub(). Idempotent on nullptr.
    LIMX_SDK_API void closePub(void *handle);

    /**
     * @brief One topic as reported by mros discovery.
     *
     * @a definition is the ROS concatenated definition when the peer filled it
     * in. Live discovery frequently leaves it empty; treat an empty string as
     * "unknown", not as "this message has no fields". Matching never uses it.
     */
    struct TopicInfo
    {
      std::string name;
      std::string type;
      std::string md5;
      std::string definition;
    };

    /**
     * @brief One live topic compared against the generated TYPE+MD5 table.
     *
     * @a mirrored is true when the SDK ships a generated class for @a type.
     * @a md5_match is true when that class's MD5 equals the peer's. A type the
     * SDK has never mirrored is ``mirrored=false, md5_match=false``. A type
     * the SDK has, but at a different revision, is ``mirrored=true,
     * md5_match=false`` — that is the ``/robot_mode`` Bool-vs-Int8 class of
     * pit, where the name is familiar but the wire schema is not this one.
     */
    struct TopicSupport
    {
      std::string name;
      std::string type;
      std::string md5;
      bool mirrored;
      bool md5_match;
    };

    /**
     * @brief Topics visible in this mros domain (any node that currently has
     *        a publisher or a subscriber).
     *
     * Requires ``Tron2::init`` / ``Robot.init`` (or any other call that has
     * already brought the mros node up). Blocks up to @p duration_sec waiting
     * for discovery; 3 s is the mros default and is an upper bound, not a
     * typical wait. Lazy channels that have not been opened yet do not appear.
     *
     * When ``MROS_DOMAIN_ID`` is a non-MAC token, mros prefixes topic names
     * with ``/did_<id>/``. A MAC-form domain id does not add that prefix
     * (and a malformed MAC can make mros ``exit(1)``).
     */
    LIMX_SDK_API std::vector<TopicInfo> getTopics(int duration_sec = 3);

    /**
     * @brief Topics that currently have a publisher in this mros domain.
     *
     * Domain-wide, not "this process only" — that is how mros reports it.
     * Same init requirement and duration behaviour as getTopics().
     */
    LIMX_SDK_API std::vector<TopicInfo> getPublishedTopics(int duration_sec = 3);

    /**
     * @brief Topics that currently have a subscriber in this mros domain.
     *
     * Domain-wide, not "this process only" — that is how mros reports it.
     * Same init requirement and duration behaviour as getTopics().
     */
    LIMX_SDK_API std::vector<TopicInfo> getSubscribedTopics(int duration_sec = 3);

    /**
     * @brief Live topic list crossed with the generated TYPE+MD5 registry.
     *
     * The registry is produced by ``tools/limxsdk-gen`` (``type_registry.h``),
     * not a hand-written table. Call after init, same as getTopics().
     */
    LIMX_SDK_API std::vector<TopicSupport> queryTopicSupport(int duration_sec = 3);

    /**
     * @brief RAII handle for one generic subscription of message type @p M.
     *
     * Move-only, because the trampoline captures the address of the state block
     * this object owns.
     */
    template <typename M>
    class Subscription
    {
    public:
      typedef std::function<void(const std::shared_ptr<const M> &)> Callback;

      Subscription() : handle_(nullptr), state_(nullptr) {}

      Subscription(Subscription &&other) : handle_(other.handle_), state_(other.state_)
      {
        other.handle_ = nullptr;
        other.state_ = nullptr;
      }

      Subscription &operator=(Subscription &&other)
      {
        if (this != &other)
        {
          reset();
          handle_ = other.handle_;
          state_ = other.state_;
          other.handle_ = nullptr;
          other.state_ = nullptr;
        }
        return *this;
      }

      Subscription(const Subscription &) = delete;
      Subscription &operator=(const Subscription &) = delete;

      ~Subscription() { reset(); }

      bool valid() const { return handle_ != nullptr; }

      /**
       * @brief Frames dropped because they did not decode as @p M.
       *
       * A non-zero count on an otherwise healthy topic points at a schema skew:
       * either an md5 collision or a peer built against a different revision of
       * the message.
       */
      uint64_t decodeErrors() const
      {
        return state_ == nullptr ? 0 : state_->decode_errors.load(std::memory_order_relaxed);
      }

      void reset()
      {
        if (handle_ != nullptr)
        {
          closeSub(handle_);
          handle_ = nullptr;
        }
        // Safe only after closeSub() has returned, which is where the no-more-
        // callbacks guarantee comes from.
        delete state_;
        state_ = nullptr;
      }

      /// @brief Open the subscription. Returns false if it could not be created.
      bool open(const std::string &topic, const Callback &cb)
      {
        reset();
        state_ = new State(cb);
        handle_ = openSub(topic.c_str(), M::TYPE, M::MD5, M::definition(), &onFrame, state_);
        if (handle_ == nullptr)
        {
          delete state_;
          state_ = nullptr;
          return false;
        }
        return true;
      }

    private:
      struct State
      {
        explicit State(const Callback &c) : cb(c), decode_errors(0) {}
        Callback cb;
        std::atomic<uint64_t> decode_errors;
      };

      static void onFrame(const unsigned char *data, int size, void *user)
      {
        State *state = static_cast<State *>(user);
        if (state == nullptr || !state->cb || data == nullptr || size < 0)
        {
          return;
        }

        std::shared_ptr<M> message = std::make_shared<M>();
        ::limxsdk::codec::Reader reader(data, static_cast<std::size_t>(size));
        // A frame is either decoded whole or dropped whole. Trailing bytes mean
        // the sender's schema is not the generated one, so that is a drop too:
        // the fields parsed so far happen to fit but cannot be trusted.
        if (!message->decode(reader) || !reader.exhausted())
        {
          state->decode_errors.fetch_add(1, std::memory_order_relaxed);
          return;
        }

        state->cb(std::shared_ptr<const M>(message));
      }

      void *handle_;
      State *state_;
    };

    /**
     * @brief Subscribe to @p topic and deliver decoded @p M messages to @p cb.
     *
     * The returned handle owns the subscription; dropping it unsubscribes.
     * Check valid() to tell a live subscription from a failed one.
     */
    template <typename M>
    inline Subscription<M> subscribe(const std::string &topic,
                                     std::function<void(const std::shared_ptr<const M> &)> cb)
    {
      Subscription<M> subscription;
      subscription.open(topic, cb);
      return subscription;
    }

    /**
     * @brief RAII handle for one generic publisher of message type @p M.
     *
     * Move-only. Dropping it unadvertises. Hold it and call publish() on each
     * frame rather than advertising per message: that is the mros/ROS contract
     * and matches how the dedicated SDK publishers (gripper, chassis, ...) work.
     */
    template <typename M>
    class Publisher
    {
    public:
      Publisher() : handle_(nullptr) {}

      Publisher(Publisher &&other) : handle_(other.handle_) { other.handle_ = nullptr; }

      Publisher &operator=(Publisher &&other)
      {
        if (this != &other)
        {
          reset();
          handle_ = other.handle_;
          other.handle_ = nullptr;
        }
        return *this;
      }

      Publisher(const Publisher &) = delete;
      Publisher &operator=(const Publisher &) = delete;

      ~Publisher() { reset(); }

      bool valid() const { return handle_ != nullptr; }

      void reset()
      {
        if (handle_ != nullptr)
        {
          closePub(handle_);
          handle_ = nullptr;
        }
      }

      /// @brief Advertise @p topic as @p M. Returns false if it could not be created.
      bool open(const std::string &topic)
      {
        reset();
        handle_ = openPub(topic.c_str(), M::TYPE, M::MD5, M::definition());
        return handle_ != nullptr;
      }

      /**
       * @brief Encode @p msg and send it. Returns false if the publisher is
       *        closed, encoding fails, or the transport rejects the frame.
       */
      bool publish(const M &msg)
      {
        if (handle_ == nullptr)
        {
          return false;
        }
        const std::size_t n = msg.encodedSize();
        std::vector<unsigned char> buffer(n);
        ::limxsdk::codec::Writer writer(n == 0 ? nullptr : &buffer[0], n);
        if (!msg.encode(writer) || writer.offset() != n)
        {
          return false;
        }
        return publishRaw(handle_, n == 0 ? nullptr : &buffer[0], static_cast<int>(n)) == 0;
      }

    private:
      void *handle_;
    };

    /**
     * @brief Advertise @p topic as message type @p M and return a live publisher.
     *
     * Named advertise (not publish) because the handle is what you hold, the
     * same way mros/ROS do it. Check valid() to tell a live publisher from a
     * failed one.
     */
    template <typename M>
    inline Publisher<M> advertise(const std::string &topic)
    {
      Publisher<M> publisher;
      publisher.open(topic);
      return publisher;
    }

  } // namespace channel
} // namespace limxsdk

#endif // _LIMX_SDK_TOPIC_CHANNEL_H_
