# Host To Device

### QUERY INTERFACE ID

    | 0x01 |

  Device replies with INERFACE ID.

### SEND MESSAGE STD ID

    | 0x02 | id_lsb | id_msb | data_length | data... |

  ID must be 0 <= id <= 0x7ff. Length must be 0 <= data_length <= 8.

### SEND MESSAGE EXT ID ('T')

    | 0x54 | id_lsb | id | id | id_msb | data_length | data... |

  ID must be 0 <= id <= 0x1fffffff. Length must be 0 <= data_length <= 8.

### SET BITRATE ('S')

    | 0x53 | bitrate |

  TBD. Currently not implemented.

### SYNCHRONIZE

    | 0xAA |

  Device replies with a series of 0xAA bytes (no fewer than 12) to ensure that any previous partially sent frame has been flushed.

### DISABLE RX

    | 0xF0 |

### ENABLE RX

    | 0xF1 |


# Device To Host

### INTERFACE ID

    | 0x01 | interface id... | \r | \n |

  Interface ID is an UTF-8 string, e.g. 'Ocarina III r1' and should only use printable characters out of the basic ASCII character set. The length of the string must be determined by finding the terminating carriage return and line break. The string itself MUST NOT contain \r or \n characters.

### RECEIVE MESSAGE STD ID

    | 0x02 | id_lsb | id_msb | data_length | data... |

### RECEIVE MESSAGE EXT ID ('R')

    | 0x52 | id_lsb | id | id | id_msb | data_length | data... |
