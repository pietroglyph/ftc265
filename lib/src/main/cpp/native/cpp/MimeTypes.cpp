// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/MimeTypes.h"

#include "wpi/StringMap.h"

namespace wpi {

// derived partially from
// https://github.com/DEGoodmanWilson/libmime/blob/stable/0.1.2/mime/mime.cpp
StringRef MimeTypeFromPath(StringRef path) {
  // https://developer.mozilla.org/en-US/docs/Web/HTTP/Basics_of_HTTP/MIME_types
  static StringMap<const char*> mimeTypes{
      // text
      {"css", "text/css"},
      {"csv", "text/csv"},
      {"htm", "text/html"},
      {"html", "text/html"},
      {"js", "text/javascript"},
      {"json", "application/json"},
      {"map", "application/json"},
      {"md", "text/markdown"},
      {"txt", "text/plain"},
      {"xml", "text/xml"},

      // images
      {"apng", "image/apng"},
      {"bmp", "image/bmp"},
      {"gif", "image/gif"},
      {"cur", "image/x-icon"},
      {"ico", "image/x-icon"},
      {"jpg", "image/jpeg"},
      {"jpeg", "image/jpeg"},
      {"png", "image/png"},
      {"svg", "image/svg+xml"},
      {"tif", "image/tiff"},
      {"tiff", "image/tiff"},
      {"webp", "image/webp"},

      // fonts
      {"otf", "font/otf"},
      {"ttf", "font/ttf"},
      {"woff", "font/woff"},

      // misc
      {"pdf", "application/pdf"},
      {"zip", "application/zip"},
  };

  static const char* defaultType = "application/octet-stream";

  auto pos = path.find_last_of("/");
  if (pos != StringRef::npos) {
    path = path.substr(pos + 1);
  }
  auto dot_pos = path.find_last_of(".");
  if (dot_pos > 0 && dot_pos != StringRef::npos) {
    auto type = mimeTypes.find(path.substr(dot_pos + 1));
    if (type != mimeTypes.end()) {
      return type->getValue();
    }
  }
  return defaultType;
}

}  // namespace wpi
