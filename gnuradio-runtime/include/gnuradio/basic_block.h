/* -*- c++ -*- */
/*
 * Copyright 2006,2008,2009,2011,2013 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_GR_BASIC_BLOCK_H
#define INCLUDED_GR_BASIC_BLOCK_H

#include <gnuradio/api.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <boost/enable_shared_from_this.hpp>
#include <map>
#include <string>

#ifdef GR_CTRLPORT
#include <gnuradio/rpcregisterhelpers.h>
#endif

namespace gr {

class endpoint;
class msg_endpoint;

/*!
 * \brief The abstract base class for all signal processing blocks.
 * \ingroup internal
 *
 * Basic blocks are the bare abstraction of an entity that has a
 * name, a set of inputs and outputs, and a message queue.  These
 * are never instantiated directly; rather, this is the abstract
 * parent class of both gr_hier_block, which is a recursive
 * container, and block, which implements actual signal
 * processing functions.
 */
class GR_RUNTIME_API basic_block : public messages::msg_accepter,
                                   public boost::enable_shared_from_this<basic_block>
{
public:
    virtual ~basic_block();
    uint64_t unique_id() const { return d_unique_id; }
    std::string unique_name() const { return d_unique_name; }

    /*! Name of the block as defined in constructor */
    std::string name() const { return d_name; }

    gr::io_signature::sptr input_signature() const { return d_input_signature; }
    gr::io_signature::sptr output_signature() const { return d_output_signature; }
    basic_block_sptr to_basic_block(); // Needed for Python type coercion

    /*!
     * True if the block has an alias (see set_block_alias).
     */
    bool alias_set() const { return !d_alias.empty(); }

    /*!
     * Returns the block's alias as a string.
     */
    std::string alias() const { return alias_set() ? d_alias : d_unique_name; }

    /*!
     * Returns the block's alias as PMT.
     */
    pmt::pmt_t alias_pmt() const { return pmt::intern(alias()); }

    /*!
     * Set's a new alias for the block; also adds an entry into the
     * block_registry to get the block using either the alias or the
     * original symbol name.
     */
    void set_block_alias(const std::string& name);

    // ** Message passing interface **

    virtual void message_port_register_in(const std::string& port_id) = 0;
    /*!
     * \brief Get input message port names.
     */
    virtual std::vector<std::string> message_ports_in() const = 0;

    void message_port_register_out(const std::string& port_id);
    void message_port_sub(const std::string& port_id, basic_block_sptr target, const std::string& target_port);
    void message_port_unsub(const std::string& port_id, basic_block_sptr target, const std::string& target_port);
    std::vector<msg_endpoint> message_subscribers(const std::string& port_id);

    /*!
     * \brief Get output message port names.
     */
    std::vector<std::string> message_ports_out() const;

#ifdef GR_CTRLPORT
    /*!
     * \brief Add an RPC variable (get or set).
     *
     * Using controlport, we create new getters/setters and need to
     * store them. Each block has a vector to do this, and these never
     * need to be accessed again once they are registered with the RPC
     * backend. This function takes a
     * boost::shared_sptr<rpcbasic_base> so that when the block is
     * deleted, all RPC registered variables are cleaned up.
     *
     * \param s an rpcbasic_sptr of the new RPC variable register to store.
     */
    void add_rpc_variable(rpcbasic_sptr s) { d_rpc_vars.push_back(s); }
#endif /* GR_CTRLPORT */

    /*!
     * \brief Set up the RPC registered variables.
     *
     * This must be overloaded by a block that wants to use
     * controlport. This is where rpcbasic_register_{get,set} pointers
     * are created, which then get wrapped as shared pointers
     * (rpcbasic_sptr(...)) and stored using add_rpc_variable.
     */
    virtual void setup_rpc(){};

    /*!
     * \brief Ask if this block has been registered to the RPC.
     *
     * We can only register a block once, so we use this to protect us
     * from calling it multiple times.
     */
    bool is_rpc_set() { return d_rpc_set; }

    /*!
     * \brief When the block is registered with the RPC, set this.
     */
    void rpc_set() { d_rpc_set = true; }

    /*!
     * \brief Confirm that ninputs and noutputs is an acceptable combination.
     *
     * \param ninputs	number of input streams connected
     * \param noutputs	number of output streams connected
     *
     * \returns true if this is a valid configuration for this block.
     *
     * This function is called by the runtime system whenever the
     * topology changes. Most classes do not need to override this.
     * This check is in addition to the constraints specified by the
     * input and output gr::io_signatures.
     */
    virtual bool check_topology(int /*ninputs*/, int /*noutputs*/) { return true; }

    virtual void set_processor_affinity(const std::vector<int>& mask) = 0;

    virtual void unset_processor_affinity() = 0;

    virtual std::vector<int> processor_affinity() = 0;

    virtual void set_log_level(std::string level) = 0;

    virtual std::string log_level() = 0;

protected:
    //! Protected constructor prevents instantiation by non-derived classes
    basic_block(void) {} // allows pure virtual interface sub-classes

    basic_block(const std::string& name,
                gr::io_signature::sptr input_signature,
                gr::io_signature::sptr output_signature);

    //! may only be called during constructor
    void set_input_signature(gr::io_signature::sptr iosig) { d_input_signature = iosig; }

    //! may only be called during constructor
    void set_output_signature(gr::io_signature::sptr iosig)
    {
        d_output_signature = iosig;
    }

    gr::io_signature::sptr d_input_signature;
    gr::io_signature::sptr d_output_signature;

    std::string d_name;
    uint64_t d_unique_id;
    std::string d_unique_name;
    std::string d_alias;

    bool d_rpc_set;
    std::vector<rpcbasic_sptr> d_rpc_vars; // container for all RPC variables

    std::map<std::string, std::vector<msg_endpoint>> d_message_subscribers;
};

inline bool operator<(basic_block_sptr lhs, basic_block_sptr rhs)
{
    return lhs->unique_id() < rhs->unique_id();
}

typedef std::vector<basic_block_sptr> basic_block_vector_t;
typedef std::vector<basic_block_sptr>::iterator basic_block_viter_t;

inline std::ostream& operator<<(std::ostream& os, basic_block_sptr basic_block)
{
    os << basic_block->unique_name();
    return os;
}

/*!
 * \brief Class representing a specific input or output graph endpoint
 * \ingroup internal
 */
class GR_RUNTIME_API endpoint
{
private:
    basic_block_sptr d_basic_block;
    int d_port;

public:
    endpoint() : d_basic_block(), d_port(0) {}
    endpoint(basic_block_sptr block, int port)
    {
        d_basic_block = block;
        d_port = port;
    }
    basic_block_sptr block() const { return d_basic_block; }
    int port() const { return d_port; }
    std::string identifier() const
    {
        return d_basic_block->alias() + ":" + std::to_string(d_port);
    };

    bool operator==(const endpoint& other) const;
};

inline bool endpoint::operator==(const endpoint& other) const
{
    return (d_basic_block == other.d_basic_block && d_port == other.d_port);
}

class GR_RUNTIME_API msg_endpoint
{
private:
    basic_block_sptr d_basic_block;
    std::string d_port;

public:
    msg_endpoint() : d_basic_block(nullptr), d_port("") {}
    msg_endpoint(basic_block_sptr block, std::string port)
    {
        d_basic_block = block;
        d_port = port;
    }
    basic_block_sptr block() const { return d_basic_block; }
    std::string port() const { return d_port; }
    std::string identifier() const
    {
        return d_basic_block->alias() + ":" + d_port;
    }

    bool operator==(const msg_endpoint& other) const;
};

inline bool msg_endpoint::operator==(const msg_endpoint& other) const
{
    return (d_basic_block == other.d_basic_block && d_port == other.d_port);
}

// Hold vectors of gr::endpoint objects
typedef std::vector<endpoint> endpoint_vector_t;
typedef std::vector<endpoint>::iterator endpoint_viter_t;


namespace gnuradio {
/*
 * \brief New!  Improved!  Standard method to get/create the
 * boost::shared_ptr for a block.
 */
template <class T>
boost::shared_ptr<T> get_initial_sptr(T* p)
{
    return boost::dynamic_pointer_cast<T, gr::basic_block>(gr::basic_block_sptr(p));
}
} // namespace gnuradio

} /* namespace gr */

#endif /* INCLUDED_GR_BASIC_BLOCK_H */
