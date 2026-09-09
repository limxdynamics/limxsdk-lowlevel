/**
 * @file codec.h
 *
 * @brief Self-contained wire codec for the LIMX SDK generic_sub_pub_topic channel.
 *
 * The on-wire layout is the ROS1 "compact little-endian" encoding that mros
 * uses: primitives are raw little-endian with no alignment padding, a variable
 * length array is a uint32 element count followed by the elements, a fixed
 * length array has no count at all, a string is a uint32 byte count followed by
 * the raw bytes, and time/duration are two 32-bit words (sec, nsec).
 *
 * This header must stay free of any mros dependency: integrators receive only
 * the headers under include/limxsdk, so anything reachable from here has to be
 * plain C++ standard library.
 *
 * © [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_CODEC_CODEC_H_
#define _LIMX_SDK_CODEC_CODEC_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#error "limxsdk::codec assumes a little-endian host, which is what the mros wire format encodes."
#endif

namespace limxsdk
{
  namespace codec
  {
    /// @brief Wire representation of the ROS `time` primitive.
    struct Time
    {
      uint32_t sec;
      uint32_t nsec;

      Time() : sec(0), nsec(0) {}
    };

    /// @brief Wire representation of the ROS `duration` primitive.
    struct Duration
    {
      int32_t sec;
      int32_t nsec;

      Duration() : sec(0), nsec(0) {}
    };

    /**
     * @brief Bounds-checked sequential reader over one received frame.
     *
     * Every read either advances the cursor or latches the reader into a failed
     * state; once failed it stays failed, so a generated decode() can chain
     * reads and test the result once at the end. A caller must treat a failed
     * reader as "discard the whole frame" - partially decoded objects are never
     * meaningful because a truncated length prefix can put the cursor anywhere.
     */
    class Reader
    {
    public:
      Reader(const unsigned char *data, std::size_t size)
          : data_(data), size_(size), offset_(0), failed_(size != 0 && data == nullptr)
      {
      }

      /// @brief Number of bytes consumed so far.
      std::size_t offset() const { return offset_; }

      /// @brief Bytes left between the cursor and the end of the frame.
      std::size_t remaining() const { return failed_ ? 0 : size_ - offset_; }

      /// @brief True once any read has run out of bytes or hit a bad length.
      bool failed() const { return failed_; }

      /**
       * @brief True when the frame was consumed exactly.
       *
       * A successful decode that leaves bytes behind means the sender's schema
       * is not the one that was generated - in practice an md5 collision or a
       * stale peer - so callers should reject the frame rather than trust it.
       */
      bool exhausted() const { return !failed_ && offset_ == size_; }

      bool read(uint8_t &v) { return raw(&v, 1); }
      bool read(int8_t &v) { return raw(&v, 1); }
      bool read(uint16_t &v) { return raw(&v, 2); }
      bool read(int16_t &v) { return raw(&v, 2); }
      bool read(uint32_t &v) { return raw(&v, 4); }
      bool read(int32_t &v) { return raw(&v, 4); }
      bool read(uint64_t &v) { return raw(&v, 8); }
      bool read(int64_t &v) { return raw(&v, 8); }
      bool read(float &v) { return raw(&v, 4); }
      bool read(double &v) { return raw(&v, 8); }

      bool read(Time &v) { return read(v.sec) && read(v.nsec); }
      bool read(Duration &v) { return read(v.sec) && read(v.nsec); }

      bool read(std::string &v)
      {
        uint32_t length = 0;
        if (!read(length))
        {
          return false;
        }
        if (length > remaining())
        {
          return fail();
        }
        v.assign(reinterpret_cast<const char *>(data_ + offset_), length);
        offset_ += length;
        return true;
      }

      /// @brief Variable-length array of a fixed-width arithmetic element.
      template <typename T>
      bool readPodArray(std::vector<T> &out)
      {
        uint32_t length = 0;
        if (!readLength(length, sizeof(T)))
        {
          return false;
        }
        out.resize(length);
        if (length != 0)
        {
          std::memcpy(&out[0], data_ + offset_, static_cast<std::size_t>(length) * sizeof(T));
          offset_ += static_cast<std::size_t>(length) * sizeof(T);
        }
        return true;
      }

      /// @brief Variable-length array of a type read through read() (string/time/duration).
      template <typename T>
      bool readValueArray(std::vector<T> &out, std::size_t min_element_size)
      {
        uint32_t length = 0;
        if (!readLength(length, min_element_size))
        {
          return false;
        }
        out.clear();
        out.resize(length);
        for (uint32_t i = 0; i < length; ++i)
        {
          if (!read(out[i]))
          {
            return false;
          }
        }
        return true;
      }

      /// @brief Variable-length array of a generated message type.
      template <typename T>
      bool readMsgArray(std::vector<T> &out)
      {
        std::size_t min_element_size = T::minEncodedSize();
        if (min_element_size == 0)
        {
          min_element_size = 1;
        }
        uint32_t length = 0;
        if (!readLength(length, min_element_size))
        {
          return false;
        }
        out.clear();
        out.resize(length);
        for (uint32_t i = 0; i < length; ++i)
        {
          if (!out[i].decode(*this))
          {
            return false;
          }
        }
        return true;
      }

      /// @brief Fixed-length array of a fixed-width arithmetic element (no length prefix).
      template <typename T, std::size_t N>
      bool readPodArrayFixed(std::array<T, N> &out)
      {
        return N == 0 ? true : raw(&out[0], N * sizeof(T));
      }

      /// @brief Fixed-length array of a type read through read() (no length prefix).
      template <typename T, std::size_t N>
      bool readValueArrayFixed(std::array<T, N> &out)
      {
        for (std::size_t i = 0; i < N; ++i)
        {
          if (!read(out[i]))
          {
            return false;
          }
        }
        return true;
      }

      /// @brief Fixed-length array of a generated message type (no length prefix).
      template <typename T, std::size_t N>
      bool readMsgArrayFixed(std::array<T, N> &out)
      {
        for (std::size_t i = 0; i < N; ++i)
        {
          if (!out[i].decode(*this))
          {
            return false;
          }
        }
        return true;
      }

    private:
      /**
       * @brief Read an element count and reject one that cannot possibly fit.
       *
       * Without this check a corrupt or mismatched frame turns a 4-byte length
       * into a multi-gigabyte resize(), so the cheapest smallest-element bound
       * is applied before any allocation happens.
       */
      bool readLength(uint32_t &length, std::size_t min_element_size)
      {
        if (!read(length))
        {
          return false;
        }
        if (min_element_size != 0 && static_cast<std::size_t>(length) > remaining() / min_element_size)
        {
          return fail();
        }
        return true;
      }

      bool raw(void *dst, std::size_t count)
      {
        if (failed_ || count > size_ - offset_)
        {
          return fail();
        }
        std::memcpy(dst, data_ + offset_, count);
        offset_ += count;
        return true;
      }

      bool fail()
      {
        failed_ = true;
        return false;
      }

      const unsigned char *data_;
      std::size_t size_;
      std::size_t offset_;
      bool failed_;
    };

    /**
     * @brief Sequential writer over a caller-sized buffer.
     *
     * The buffer is expected to be exactly encodedSize() bytes, so an overflow
     * here means the size calculation and the encoder disagree; that latches the
     * writer into a failed state instead of writing out of bounds.
     */
    class Writer
    {
    public:
      Writer(unsigned char *data, std::size_t size)
          : data_(data), size_(size), offset_(0), failed_(size != 0 && data == nullptr)
      {
      }

      std::size_t offset() const { return offset_; }
      bool failed() const { return failed_; }

      bool write(uint8_t v) { return raw(&v, 1); }
      bool write(int8_t v) { return raw(&v, 1); }
      bool write(uint16_t v) { return raw(&v, 2); }
      bool write(int16_t v) { return raw(&v, 2); }
      bool write(uint32_t v) { return raw(&v, 4); }
      bool write(int32_t v) { return raw(&v, 4); }
      bool write(uint64_t v) { return raw(&v, 8); }
      bool write(int64_t v) { return raw(&v, 8); }
      bool write(float v) { return raw(&v, 4); }
      bool write(double v) { return raw(&v, 8); }

      bool write(const Time &v) { return write(v.sec) && write(v.nsec); }
      bool write(const Duration &v) { return write(v.sec) && write(v.nsec); }

      bool write(const std::string &v)
      {
        if (!write(static_cast<uint32_t>(v.size())))
        {
          return false;
        }
        return v.empty() ? true : raw(v.data(), v.size());
      }

      template <typename T>
      bool writePodArray(const std::vector<T> &in)
      {
        if (!write(static_cast<uint32_t>(in.size())))
        {
          return false;
        }
        return in.empty() ? true : raw(&in[0], in.size() * sizeof(T));
      }

      template <typename T>
      bool writeValueArray(const std::vector<T> &in)
      {
        if (!write(static_cast<uint32_t>(in.size())))
        {
          return false;
        }
        for (std::size_t i = 0; i < in.size(); ++i)
        {
          if (!write(in[i]))
          {
            return false;
          }
        }
        return true;
      }

      template <typename T>
      bool writeMsgArray(const std::vector<T> &in)
      {
        if (!write(static_cast<uint32_t>(in.size())))
        {
          return false;
        }
        for (std::size_t i = 0; i < in.size(); ++i)
        {
          if (!in[i].encode(*this))
          {
            return false;
          }
        }
        return true;
      }

      template <typename T, std::size_t N>
      bool writePodArrayFixed(const std::array<T, N> &in)
      {
        return N == 0 ? true : raw(&in[0], N * sizeof(T));
      }

      template <typename T, std::size_t N>
      bool writeValueArrayFixed(const std::array<T, N> &in)
      {
        for (std::size_t i = 0; i < N; ++i)
        {
          if (!write(in[i]))
          {
            return false;
          }
        }
        return true;
      }

      template <typename T, std::size_t N>
      bool writeMsgArrayFixed(const std::array<T, N> &in)
      {
        for (std::size_t i = 0; i < N; ++i)
        {
          if (!in[i].encode(*this))
          {
            return false;
          }
        }
        return true;
      }

    private:
      bool raw(const void *src, std::size_t count)
      {
        if (failed_ || count > size_ - offset_)
        {
          failed_ = true;
          return false;
        }
        std::memcpy(data_ + offset_, src, count);
        offset_ += count;
        return true;
      }

      unsigned char *data_;
      std::size_t size_;
      std::size_t offset_;
      bool failed_;
    };

    /// @brief Encoded size of one string element, including its length prefix.
    inline std::size_t elementSize(const std::string &v) { return 4 + v.size(); }
    inline std::size_t elementSize(const Time &) { return 8; }
    inline std::size_t elementSize(const Duration &) { return 8; }

    /// @brief Encoded size of one generated message element.
    template <typename T>
    inline std::size_t elementSize(const T &v)
    {
      return v.encodedSize();
    }

    /**
     * @brief Summed encoded size of every element of an array.
     *
     * Excludes the length prefix, which the generated encodedSize() accounts for
     * separately so that fixed-length arrays can reuse this helper.
     */
    template <typename Container>
    inline std::size_t arraySize(const Container &c)
    {
      std::size_t total = 0;
      for (typename Container::const_iterator it = c.begin(); it != c.end(); ++it)
      {
        total += elementSize(*it);
      }
      return total;
    }

  } // namespace codec
} // namespace limxsdk

#endif // _LIMX_SDK_CODEC_CODEC_H_
