package cm

type contactBuffer struct {
	// header
	stamp       uint
	next        *contactBuffer
	numContacts int

	// buffer itself
	contacts [ContactsBufferSize]Contact
}

func NewContactBuffer(stamp uint, slice *contactBuffer) *contactBuffer {
	buffer := &contactBuffer{}
	buffer.contacts = [ContactsBufferSize]Contact{}
	return buffer.InitHeader(stamp, slice)
}

func (c *contactBuffer) InitHeader(stamp uint, splice *contactBuffer) *contactBuffer {
	c.stamp = stamp
	if splice != nil {
		c.next = splice.next
	} else {
		c.next = c
	}
	c.numContacts = 0

	return c
}
