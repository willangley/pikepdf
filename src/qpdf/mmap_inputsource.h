/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (C) 2017, James R. Barlow (https://github.com/jbarlow83/)
 */


#include <cstdio>
#include <cstring>

#include <qpdf/Constants.h>
#include <qpdf/Types.h>
#include <qpdf/DLL.h>
#include <qpdf/QPDFExc.hh>
#include <qpdf/PointerHolder.hh>
#include <qpdf/Buffer.hh>
#include <qpdf/QPDF.hh>
#include <qpdf/InputSource.hh>
#include <qpdf/QUtil.hh>
#include <qpdf/Buffer.hh>
#include <qpdf/BufferInputSource.hh>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pikepdf.h"

#include <sys/mman.h>
#include <sys/stat.h>


class MmapInputSource : public InputSource
{
public:
    MmapInputSource(py::object stream, const std::string& description, bool close_stream) :
            InputSource(), stream(stream), description(description), close_stream(close_stream), offset(0)
    {
        py::int_ fileno = stream.attr("fileno")();
        int fd = fileno;
        auto mmap_module = py::module::import("mmap");
        auto mmap_fn = mmap_module.attr("mmap");
        auto prot_read = mmap_module.attr("PROT_READ");
        auto flags_shared = mmap_module.attr("MAP_SHARED");
        this->mmap = mmap_fn(fd, 0, flags_shared, prot_read);
        py::buffer view(this->mmap);
        this->buffer_info = std::make_unique<py::buffer_info>(view.request(false));
    }
    virtual ~MmapInputSource()
    {
        py::gil_scoped_acquire gil;
        this->buffer_info.reset();
        if (!this->mmap.is_none()) {
            this->mmap.attr("close")();
        }
        if (this->close_stream) {
            this->stream.attr("close")();
        }
    }
    MmapInputSource(const MmapInputSource&) = delete;
    MmapInputSource& operator= (const MmapInputSource&) = delete;
    MmapInputSource(MmapInputSource&&) = delete;
    MmapInputSource& operator= (MmapInputSource&&) = delete;

    std::string const& getName() const override
    {
        return this->description;
    }

    qpdf_offset_t tell() override
    {
        return this->offset;
    }

    void seek(qpdf_offset_t offset, int whence) override
    {
        switch(whence)
        {
            case SEEK_SET:
                this->offset = offset;
                break;
            case SEEK_END:
                this->offset = this->buffer_info->size + offset;
                break;
            case SEEK_CUR:
                this->offset += offset;
                break;
            default:
	            throw std::logic_error(
	                "INTERNAL ERROR: invalid argument to MmapInputSource::seek"
                );
	            break;
        }
        if (this->offset < 0)
        {
            throw std::runtime_error(
                this->description + ": seek before beginning of buffer");
        }
    }

    void rewind() override
    {
        this->seek(0, SEEK_SET);
    }

    size_t read(char* buffer, size_t length) override
    {
        if (this->offset < 0)
        {
            throw std::logic_error("INTERNAL ERROR: BufferInputSource offset < 0");
        }
        qpdf_offset_t end_pos = this->buffer_info->size;
        if (this->offset >= end_pos)
        {
            this->last_offset = end_pos;
            return 0;
        }

        this->last_offset = this->offset;
        size_t len = std::min(
            QIntC::to_size(end_pos - this->offset), length);
        memcpy(buffer, static_cast<const char*>(this->buffer_info->ptr) + this->offset, len);
        this->offset += QIntC::to_offset(len);
        return len;
    }

    void unreadCh(char ch) override
    {
        if (this->offset > 0) {
            --this->offset;
        }
    }

    qpdf_offset_t findAndSkipNextEOL() override
    {
        if (this->offset < 0)
        {
            throw std::logic_error("INTERNAL ERROR: BufferInputSource offset < 0");
        }
        qpdf_offset_t end_pos = this->buffer_info->size;
        if (this->offset >= end_pos)
        {
            this->last_offset = end_pos;
            this->offset = end_pos;
            return end_pos;
        }
        qpdf_offset_t result = 0;
        unsigned char const* buffer = static_cast<unsigned char const*>(this->buffer_info->ptr);

        unsigned char const* end = buffer + end_pos;
        unsigned char const* p = buffer + this->offset;

        while (p < end) {
            if (*p == '\r' || *p == '\n')
                break;
            ++p;
        }
        if (p != end) {
            result = p - buffer;
            this->offset = result + 1;
            ++p;
            while ((this->offset < end_pos) &&
                ((*p == '\r') || (*p == '\n')))
            {
                ++p;
                ++this->offset;
            }
        }
        else
        {
            this->offset = end_pos;
            result = end_pos;
        }
        return result;
    }

private:
    py::object stream;
    std::string description;
    bool close_stream;
    py::object mmap;
    std::unique_ptr<py::buffer_info> buffer_info;
    qpdf_offset_t offset;
};
