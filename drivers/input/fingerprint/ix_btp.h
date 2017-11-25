#ifndef LINUX_SPI_IX_H
#define LINUX_SPI_IX_H

struct ix_platform_data {
    int irq_gpio;
    int reset_gpio;
    int cs_gpio;
	int qup_id;
    struct regulator *vreg;
};

#endif
