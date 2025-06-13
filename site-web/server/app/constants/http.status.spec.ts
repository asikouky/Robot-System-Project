import {
    HTTP_STATUS_SERVER_ERROR,
    HTTP_STATUS_OK,
    HTTP_STATUS_CREATED,
    HTTP_STATUS_NO_CONTENT,
    HTTP_STATUS_NOT_FOUND,
    HTTP_STATUS_CONFLICT,
    HTTP_STATUS_BAD_REQUEST
  } from './http.status';
  
  import { StatusCodes } from 'http-status-codes';
  import { expect } from 'chai';
  
  describe('HTTP status constants', () => {
    it('should match values from http-status-codes', () => {
      expect(HTTP_STATUS_SERVER_ERROR).to.equal(StatusCodes.INTERNAL_SERVER_ERROR);
      expect(HTTP_STATUS_OK).to.equal(StatusCodes.OK);
      expect(HTTP_STATUS_CREATED).to.equal(StatusCodes.CREATED);
      expect(HTTP_STATUS_NO_CONTENT).to.equal(StatusCodes.NO_CONTENT);
      expect(HTTP_STATUS_NOT_FOUND).to.equal(StatusCodes.NOT_FOUND);
      expect(HTTP_STATUS_CONFLICT).to.equal(StatusCodes.CONFLICT);
      expect(HTTP_STATUS_BAD_REQUEST).to.equal(StatusCodes.BAD_REQUEST);
    });
  });
  