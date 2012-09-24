#ifndef MMC_QUEUE_H
#define MMC_QUEUE_H

struct request;
struct task_struct;

struct mmc_queue {
	struct mmc_card		*card;
	struct task_struct	*thread;
	struct semaphore	thread_sem;
	unsigned int		flags;
	struct request		*req;
	int			(*issue_fn)(struct mmc_queue *, struct request *);
	void			*data;
	struct request_queue	*queue;
	struct scatterlist	*sg;
	char			*bounce_buf;
	struct scatterlist	*bounce_sg;
	unsigned int		bounce_sg_len;
};

enum mmc_blk_status {
	MMC_BLK_SUCCESS = 0,
	MMC_BLK_PARTIAL,
	MMC_BLK_CMD_ERR,
	MMC_BLK_RETRY,
	MMC_BLK_ABORT,
	MMC_BLK_DATA_ERR,
	MMC_BLK_ECC_ERR,
	MMC_BLK_NOMEDIUM,
};

extern int mmc_init_queue(struct mmc_queue *, struct mmc_card *, spinlock_t *,
			  const char *);
extern void mmc_cleanup_queue(struct mmc_queue *);
extern void mmc_queue_suspend(struct mmc_queue *);
extern void mmc_queue_resume(struct mmc_queue *);

extern unsigned int mmc_queue_map_sg(struct mmc_queue *);
extern void mmc_queue_bounce_pre(struct mmc_queue *);
extern void mmc_queue_bounce_post(struct mmc_queue *);

#endif
